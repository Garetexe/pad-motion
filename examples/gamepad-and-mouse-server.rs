use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Mutex;
use std::time::{Instant, Duration};
use std::thread;
use std::fs;

use gilrs::{Gilrs, Button, Axis};
use multiinput::{RawInputManager, RawEvent};

use pad_motion::protocol::*;
use pad_motion::server::*;

// --- Constantes de temporización ---
// dt mínimo permitido para el cálculo de velocidad angular. Evita que un dt
// casi-cero (jitter del scheduler / resolución del timer de Windows) dispare
// el yaw/pitch a valores absurdos al dividir por un número minúsculo.
const MIN_DT: f32 = 0.0005; // 0.5 ms
// dt máximo permitido. Evita un salto brusco si el loop se pausó
// (ventana sin foco, hitch del SO, etc).
const MAX_DT: f32 = 0.05; // 50 ms
// dt "de referencia" contra el que se calibró originalmente el parámetro
// `smoothing` (pensado para un tick nominal de ~1ms). Se usa para hacer
// el suavizado EMA independiente del framerate real.
const REFERENCE_DT: f32 = 0.001;

// Default Configuration
struct AppConfig {
    sensitivity: f32,
    invert_x: f32,      // 1.0 o -1.0
    invert_y: f32,      // 1.0 o -1.0
    gravity_axis: u8,   // 0=X, 1=Y, 2=Z
    gravity_amount: f32, // Usually 9.81
    smoothing: f32,      // 0.0 = sin suavizado, 0.98 = muy suave (interpretado como retención por REFERENCE_DT)
    deadzone: f32,       // Counts de mouse por debajo de este valor se ignoran (elimina jitter/deriva)
    pitch_limit: f32,    // Tope blando de inclinación acumulada; evita el "flip" al pasar de vertical
    pitch_recenter: f32, // Velocidad de recentrado automático del pitch (0 = desactivado)
}

impl Default for AppConfig {
    fn default() -> Self {
        AppConfig {
            // Antes: 32.0. Más sensibilidad todavía.
            sensitivity: 40.0,
            invert_x: 1.0,  // Invertido respecto a la versión anterior (ejes estaban al revés)
            invert_y: -1.0, // Invertido respecto a la versión anterior (ejes estaban al revés)
            gravity_axis: 1, // 1 = Y-Axis (Upright/Remote style) to fix "X" movement
            gravity_amount: 9.81,
            // Subido de 0.85 a 0.9: casi el máximo de fluidez práctico (0.98 es el tope,
            // pero ahí ya se siente con demasiado retardo). Ajustable en config.txt.
            smoothing: 0.9,
            // Ignora movimientos de mouse menores a 2 counts por tick (ruido del sensor).
            // Súbelo en config.txt (ej. deadzone=5) si notás deriva/temblor con el mouse quieto.
            deadzone: 2.0,
            // Tope "blando": cuando el pitch acumulado llega a este valor, se deja de
            // seguir empujando en esa dirección. Está en las mismas unidades arbitrarias
            // que gyroscope_pitch (no son grados reales), así que se calibra a ojo:
            // si todavía llegás a voltear el control, bajalo (ej. pitch_limit=200).
            pitch_limit: 300.0,
            // Recentra el pitch acumulado hacia 0 automáticamente cuando no hay
            // movimiento vertical nuevo. Esto es la "auto-calibración": evita que
            // la deriva se vaya acercando al límite sin que te des cuenta.
            pitch_recenter: 0.15,
        }
    }
}

fn main() {
  let running = Arc::new(AtomicBool::new(true));

  {
    let running = running.clone();
    ctrlc::set_handler(move || {
      running.store(false, Ordering::SeqCst);
    }).expect("Error setting Ctrl-C handler");
  }

  let server = Arc::new(Server::new(None, None).unwrap());
  let server_thread_join_handle = {
    let server = server.clone();
    server.start(running.clone())
  };

  let controller_info = ControllerInfo {
    slot_state: SlotState::Connected,
    device_type: DeviceType::FullGyro,
    connection_type: ConnectionType::USB,
    .. Default::default()
  };
  server.update_controller_info(controller_info);

  fn to_stick_value(input: f32) -> u8 {
    (input * 127.0 + 127.0) as u8 
  }

  // Shared Config (Thread Safe)
  let config = Arc::new(Mutex::new(AppConfig::default()));

  // --- CONFIG FILE WATCHER ---
  // Reads 'config.txt' every second.
  // Format: key=value (e.g., sensitivity=5.0)
  {
      let config = config.clone();
      let running = running.clone();
      thread::spawn(move || {
          while running.load(Ordering::Relaxed) {
              thread::sleep(Duration::from_secs(1));
              
              if let Ok(contents) = fs::read_to_string("config.txt") {
                  let mut new_config = AppConfig::default(); // Reset to defaults first
                  
                  for line in contents.lines() {
                      if let Some((key, value)) = line.split_once('=') {
                          let key = key.trim();
                          let val = value.trim().parse::<f32>().unwrap_or(0.0);
                          
                          match key {
                              "sensitivity" => new_config.sensitivity = val,
                              "invert_x" => new_config.invert_x = if val > 0.0 { 1.0 } else { -1.0 },
                              "invert_y" => new_config.invert_y = if val > 0.0 { 1.0 } else { -1.0 },
                              "gravity_axis" => new_config.gravity_axis = val as u8,
                              "gravity_amount" => new_config.gravity_amount = val,
                              "smoothing" => new_config.smoothing = val.clamp(0.0, 0.98),
                              "deadzone" => new_config.deadzone = val.max(0.0),
                              "pitch_limit" => new_config.pitch_limit = val.max(1.0),
                              "pitch_recenter" => new_config.pitch_recenter = val.max(0.0),
                              _ => {}
                          }
                      }
                  }
                  
                  // Update the shared config
                  println!(
                      "[config.txt recargado] sensibilidad={:.1}  suavizado={:.2}  invert_x={}  invert_y={}  gravity_axis={}  deadzone={:.1}  pitch_limit={:.0}  pitch_recenter={:.2}",
                      new_config.sensitivity, new_config.smoothing, new_config.invert_x, new_config.invert_y, new_config.gravity_axis,
                      new_config.deadzone, new_config.pitch_limit, new_config.pitch_recenter
                  );
                  if let Ok(mut c) = config.lock() {
                      *c = new_config;
                  }
              }
          }
      });
  }

  let mut gilrs = Gilrs::new().unwrap();
  let mut mouse_manager = RawInputManager::new().unwrap();
  mouse_manager.register_devices(multiinput::DeviceType::Mice);

  let now = Instant::now();

  // Estado persistente para el suavizado (EMA) y el cálculo de dt real
  let mut smoothed_yaw: f32 = 0.0;
  let mut smoothed_pitch: f32 = 0.0;
  // Estimación acumulada de "cuánto se inclinó" en el eje vertical, usada
  // sólo para el tope blando y el auto-recentrado (ver más abajo).
  let mut pitch_accum: f32 = 0.0;
  let mut last_tick = Instant::now();

  while running.load(Ordering::SeqCst) {
    // Consume controller events
    while let Some(_event) = gilrs.next_event() {
    }

    let mut delta_rotation_x = 0.0;
    let mut delta_rotation_y = 0.0;
    
    while let Some(event) = mouse_manager.get_event() {
      match event {
        RawEvent::MouseMoveEvent(_mouse_id, delta_x, delta_y) => {
          delta_rotation_x += delta_x as f32;
          delta_rotation_y += delta_y as f32;
        },
        _ => ()
      }
    }

    // Tiempo real transcurrido desde la última vuelta del loop.
    // Se limita (clamp) a un rango razonable: esto es clave para eliminar
    // el movimiento "a tropicones", ya que en Windows thread::sleep(1ms)
    // no es preciso y dt puede oscilar erráticamente entre ~0.5ms y ~15ms+.
    // Sin este clamp, un dt casi-cero dispara el yaw/pitch a valores enormes.
    let dt_raw = last_tick.elapsed().as_secs_f32();
    let dt = dt_raw.clamp(MIN_DT, MAX_DT);
    last_tick = Instant::now();

    // Capture current config snapshot
    let (sens, inv_x, inv_y, g_axis, g_val, smooth, deadzone, pitch_limit, pitch_recenter) = {
        let c = config.lock().unwrap();
        (c.sensitivity, c.invert_x, c.invert_y, c.gravity_axis, c.gravity_amount, c.smoothing,
         c.deadzone, c.pitch_limit, c.pitch_recenter)
    };

    // --- Zona muerta: ignora micro-movimientos/ruido del mouse ---
    if delta_rotation_x.abs() < deadzone { delta_rotation_x = 0.0; }
    if delta_rotation_y.abs() < deadzone { delta_rotation_y = 0.0; }

    // Normaliza por dt (velocidad angular consistente sin importar
    // si el loop tardó más o menos en esta vuelta)
    let target_yaw       = (delta_rotation_x / dt) * sens * inv_x * 0.001;
    let mut target_pitch = (delta_rotation_y / dt) * sens * inv_y * 0.001;

    // --- Tope blando de pitch + auto-recentrado (evita el "flip") ---
    // El juego que recibe estos datos integra gyroscope_pitch para saber cuánto
    // te inclinaste. Si eso pasa de ~90°, empieza a leer pitch/yaw invertidos
    // (el clásico flip de un puntero/Wiimote al pasar de la vertical). Para
    // evitarlo llevamos una cuenta propia (pitch_accum) y:
    //   1) si el movimiento actual seguiría empujando pitch_accum más allá del
    //      límite, lo frenamos ahí (como si topara con algo, no rebota ni invierte),
    //   2) cuando no estás moviendo el mouse verticalmente, pitch_accum se va
    //      recentrando solo hacia 0 (auto-calibración: nunca se acumula deriva
    //      sin que te des cuenta y te acerque al límite).
    let projected_pitch = pitch_accum + target_pitch * dt;
    if projected_pitch.abs() > pitch_limit && projected_pitch.signum() == target_pitch.signum() {
        target_pitch = 0.0;
    }
    pitch_accum = (pitch_accum + target_pitch * dt).clamp(-pitch_limit, pitch_limit);
    if pitch_recenter > 0.0 {
        pitch_accum *= (1.0 - pitch_recenter * dt).max(0.0);
    }

    // Suavizado exponencial (EMA) independiente del framerate real.
    // En vez de un factor fijo (1 - smoothing), se calcula la fracción de
    // retención elevándola a (dt / REFERENCE_DT). Así, si el loop tarda más
    // o menos que el tick nominal de referencia, la "fuerza" efectiva del
    // suavizado se mantiene constante en el tiempo, en vez de fluctuar con
    // el jitter del scheduler (causa principal del efecto "a tropicones").
    let alpha = 1.0 - smooth.powf(dt / REFERENCE_DT);
    smoothed_yaw   += (target_yaw   - smoothed_yaw)   * alpha;
    smoothed_pitch += (target_pitch - smoothed_pitch) * alpha;

    let gyro_yaw = smoothed_yaw;
    let gyro_pitch = smoothed_pitch;

    // Apply Gravity Vector (Fixes the "X vs +" rotation issue)
    let (accel_x, accel_y, accel_z) = match g_axis {
        0 => (g_val, 0.0, 0.0), // X-Axis (Sideways)
        1 => (0.0, g_val, 0.0), // Y-Axis (Upright/Pointer) <- DEFAULT
        _ => (0.0, 0.0, g_val), // Z-Axis (Flat)
    };

    let first_gamepad = gilrs.gamepads().next();
    let controller_data = {
      if let Some((_id, gamepad)) = first_gamepad {
        let analog_button_value = |button| {
          gamepad.button_data(button).map(|data| (data.value() * 255.0) as u8).unwrap_or(0)
        };

        ControllerData {
          connected: true,
          d_pad_left: gamepad.is_pressed(Button::DPadLeft),
          d_pad_down: gamepad.is_pressed(Button::DPadDown),
          d_pad_right: gamepad.is_pressed(Button::DPadRight),
          d_pad_up: gamepad.is_pressed(Button::DPadUp),
          start: gamepad.is_pressed(Button::Start),
          right_stick_button: gamepad.is_pressed(Button::RightThumb),
          left_stick_button: gamepad.is_pressed(Button::LeftThumb),
          select:  gamepad.is_pressed(Button::Select),
          triangle: gamepad.is_pressed(Button::North),
          circle: gamepad.is_pressed(Button::East),
          cross: gamepad.is_pressed(Button::South),
          square: gamepad.is_pressed(Button::West),
          r1: gamepad.is_pressed(Button::RightTrigger),
          l1: gamepad.is_pressed(Button::LeftTrigger),
          r2: gamepad.is_pressed(Button::RightTrigger2),
          l2: gamepad.is_pressed(Button::LeftTrigger2),
          ps: analog_button_value(Button::Mode),
          left_stick_x: to_stick_value(gamepad.value(Axis::LeftStickX)),
          left_stick_y: to_stick_value(gamepad.value(Axis::LeftStickY)),
          right_stick_x: to_stick_value(gamepad.value(Axis::RightStickX)),
          right_stick_y: to_stick_value(gamepad.value(Axis::RightStickY)),
          analog_d_pad_left: analog_button_value(Button::DPadLeft),
          analog_d_pad_down: analog_button_value(Button::DPadDown),
          analog_d_pad_right: analog_button_value(Button::DPadRight),
          analog_d_pad_up: analog_button_value(Button::DPadUp),
          analog_triangle: analog_button_value(Button::North),
          analog_circle: analog_button_value(Button::East),
          analog_cross: analog_button_value(Button::South),
          analog_square: analog_button_value(Button::West),
          analog_r1: analog_button_value(Button::RightTrigger),
          analog_l1: analog_button_value(Button::LeftTrigger),
          analog_r2: analog_button_value(Button::RightTrigger2),
          analog_l2: analog_button_value(Button::LeftTrigger2),
          motion_data_timestamp: now.elapsed().as_micros() as u64,
          
          accelerometer_x: accel_x,
          accelerometer_y: accel_y,
          accelerometer_z: accel_z,
          
          gyroscope_pitch: gyro_pitch,
          gyroscope_yaw: gyro_yaw,
          gyroscope_roll: 0.0,

          .. Default::default()
        }
      } else {
        ControllerData {
          connected: true,
          motion_data_timestamp: now.elapsed().as_micros() as u64,
          
          accelerometer_x: accel_x,
          accelerometer_y: accel_y,
          accelerometer_z: accel_z,

          gyroscope_pitch: gyro_pitch,
          gyroscope_yaw: gyro_yaw,
          gyroscope_roll: 0.0,

          .. Default::default()
        }
      }
    };

    server.update_controller_data(0, controller_data);
    std::thread::sleep(Duration::from_millis(1));
  }

  server_thread_join_handle.join().unwrap();
}