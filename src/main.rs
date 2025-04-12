use axum::{
    extract::{Path, State},
    http::StatusCode,
    response::{IntoResponse, Response},
    routing::{get, post},
    Json, Router,
};
use lx16a_rs::{Result as LX16AResult, LX16A};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::time::Duration;
use tokio;

// Define the data structures for API requests and responses
#[derive(Deserialize)]
struct SetPositionRequest {
    position: f32,
    time: i32,
}

#[derive(Serialize)]
struct ServoInfo {
    id: u8,
    temperature: i32,
    voltage: i32,
    position: f32,
}

#[derive(Serialize)]
struct ServoPositionInfo {
    id: u8,
    position: f32,
}

#[derive(Serialize)]
struct AllServosInfo {
    servos: Vec<ServoInfo>,
}

struct AppState {
    servos: Mutex<HashMap<u8, LX16A>>,
    controller: Arc<Mutex<lx16a_rs::Controller>>,
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Parse command line arguments
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 2 {
        eprintln!("Usage: {} <serial_port>", args[0]);
        std::process::exit(1);
    }
    let port = &args[1];
    println!("Initializing LX16A servo HTTP server on port {}", port);

    // Initialize the controller with a timeout of 20ms
    let controller = LX16A::initialize(port, Duration::from_millis(20))?;

    // Create a map to store multiple servos
    let mut servo_map = HashMap::new();

    // Create shared state
    let state = Arc::new(AppState {
        servos: Mutex::new(servo_map),
        controller: Arc::new(Mutex::new(controller)),
    });

    // Initialize available servos (IDs 1-12)
    // for id in 1..=12 {
    for id in [10, 11, 12] {
        if let Ok(servo) = initialize_servo(&state, id) {
            println!("Servo ID {} initialized", id);
        } else {
            println!("Servo ID {} not connected or failed to initialize", id);
        }
    }

    // Build our application with routes
    let app = Router::new()
        .route("/servos", get(handle_get_all_servos))
        .route("/servos/:id", get(handle_get_servo))
        .route("/servos/:id/position", post(handle_set_position))
        .with_state(state);

    // Start the server
    // let addr = std::net::SocketAddr::from(([127, 0, 0, 1], 3030));
    let addr = std::net::SocketAddr::from(([0, 0, 0, 0], 3030));
    println!("HTTP server listening on http://{}", addr);
    println!("Available endpoints:");
    println!("  GET  /servos         - Get info for all connected servos");
    println!("  GET  /servos/:id     - Get info for a specific servo");
    println!("  POST /servos/:id/position - Set position for a specific servo");

    axum::Server::bind(&addr)
        .serve(app.into_make_service())
        .await
        .unwrap();

    Ok(())
}

fn initialize_servo(state: &Arc<AppState>, id: u8) -> LX16AResult<()> {
    let controller_clone = state.controller.lock().unwrap().clone();
    let servo = LX16A::new(id, controller_clone, false)?;

    // Try to get some basic info to verify the servo is responding
    let _id = servo.get_id(false)?;

    // If we got here, the servo is responding, so add it to our map
    state.servos.lock().unwrap().insert(id, servo);
    Ok(())
}

fn get_servo_info(servos: &Mutex<HashMap<u8, LX16A>>, id: u8) -> Result<ServoInfo, AppError> {
    let servo_map = servos.lock().unwrap();

    let servo = servo_map
        .get(&id)
        .ok_or_else(|| AppError::NotFound(format!("Servo with ID {} not found", id)))?;

    let id = match servo.get_id(false) {
        Ok(id) => id,
        Err(e) => return Err(AppError::ServoError(format!("Error getting ID: {:?}", e))),
    };

    let temperature = match servo.get_temp() {
        Ok(temp) => temp,
        Err(e) => {
            return Err(AppError::ServoError(format!(
                "Error getting temperature: {:?}",
                e
            )))
        }
    };

    let voltage = match servo.get_vin() {
        Ok(vin) => vin,
        Err(e) => {
            return Err(AppError::ServoError(format!(
                "Error getting voltage: {:?}",
                e
            )))
        }
    };

    let position = match servo.get_physical_angle() {
        Ok(angle) => angle,
        Err(e) => {
            return Err(AppError::ServoError(format!(
                "Error getting position: {:?}",
                e
            )))
        }
    };

    Ok(ServoInfo {
        id,
        temperature,
        voltage,
        position,
    })
}

// get servo posotions as fast as possible
fn get_servo_position_info(
    servos: &Mutex<HashMap<u8, LX16A>>,
    id: u8,
) -> Result<ServoPositionInfo, AppError> {
    let servo_map = servos.lock().unwrap();

    let servo = servo_map
        .get(&id)
        .ok_or_else(|| AppError::NotFound(format!("Servo with ID {} not found", id)))?;

    let position = match servo.get_physical_angle() {
        Ok(angle) => angle,
        Err(e) => {
            return Err(AppError::ServoError(format!(
                "Error getting position: {:?}",
                e
            )))
        }
    };

    Ok(ServoPositionInfo { id, position })
}

// Error type for our application
#[derive(Debug)]
enum AppError {
    ServoError(String),
    NotFound(String),
}

impl IntoResponse for AppError {
    fn into_response(self) -> Response {
        let (status, message) = match self {
            AppError::ServoError(message) => (StatusCode::INTERNAL_SERVER_ERROR, message),
            AppError::NotFound(message) => (StatusCode::NOT_FOUND, message),
        };

        (status, message).into_response()
    }
}

async fn handle_get_all_servos(
    State(state): State<Arc<AppState>>,
) -> Result<Json<AllServosInfo>, AppError> {
    println!("Getting info for all servos");
    let servo_map = state.servos.lock().unwrap();
    let servo_ids: Vec<u8> = servo_map.keys().cloned().collect();
    drop(servo_map); // Release the lock before the loop

    let mut servo_info_list = Vec::new();
    for id in servo_ids {
        match get_servo_info(&state.servos, id) {
            Ok(info) => servo_info_list.push(info),
            Err(e) => {
                eprintln!("Error getting info for servo {}: {:?}", id, e);
                // Continue with other servos even if one fails
            }
        }
    }

    // sort the servo info list by ID
    servo_info_list.sort_by_key(|info| info.id);

    Ok(Json(AllServosInfo {
        servos: servo_info_list,
    }))
}

async fn handle_get_servo(
    State(state): State<Arc<AppState>>,
    Path(id): Path<u8>,
) -> Result<Json<ServoInfo>, AppError> {
    println!("Getting info for servo ID {}", id);
    let info = get_servo_info(&state.servos, id)?;
    Ok(Json(info))
}

async fn handle_set_position(
    State(state): State<Arc<AppState>>,
    Path(id): Path<u8>,
    Json(body): Json<SetPositionRequest>,
) -> Result<StatusCode, AppError> {
    let position = body.position;
    let time = body.time;

    let mut servo_map = state.servos.lock().unwrap();

    let servo = servo_map
        .get_mut(&id)
        .ok_or_else(|| AppError::NotFound(format!("Servo with ID {} not found", id)))?;

    match servo.move_servo(position, time, false, false) {
        Ok(_) => {
            println!(
                "Moved servo ID {} to position {}° in {} ms",
                id, position, time
            );
            Ok(StatusCode::OK)
        }
        Err(e) => {
            let error_msg = format!("Error setting servo position: {:?}", e);
            eprintln!("{}", error_msg);
            Err(AppError::ServoError(error_msg))
        }
    }
}
