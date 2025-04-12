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
use std::time::{Duration, Instant};
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
    is_moving: bool,
}

#[derive(Serialize)]
struct ServoPositionInfo {
    id: u8,
    position: f32,
    is_moving: bool,
}

#[derive(Serialize)]
struct AllServosInfo {
    servos: Vec<ServoInfo>,
}

#[derive(Serialize)]
struct AllServosPositionInfo {
    servos: Vec<ServoPositionInfo>,
}

// Struct to track movement state
struct MovementState {
    start_position: f32,
    target_position: f32,
    start_time: Instant,
    movement_duration_ms: i32,
}

// Add a new struct to store cached servo information with movement tracking
struct CachedServo {
    servo: LX16A,
    cached_position: f32,
    last_update: Instant,
    real_reads_count: usize,
    cache_hits_count: usize,
    movement: Option<MovementState>,
}

struct AppState {
    servos: Mutex<HashMap<u8, CachedServo>>,
    controller: Arc<Mutex<lx16a_rs::Controller>>,
    cache_ttl: Duration,        // How long to trust the cache before refreshing
    force_read_interval: usize, // Force a real read every N position requests
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Parse command line arguments
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 2 {
        eprintln!("Usage: {} <serial_port> [--serve | --bench]", args[0]);
        eprintln!("  --serve    Start the HTTP server (default if no mode is specified)");
        eprintln!("  --bench    Run benchmark on servo angle reading");
        std::process::exit(1);
    }

    let port = &args[1];

    // Default mode is serve
    let mode = if args.len() >= 3 { &args[2] } else { "--serve" };

    println!("Initializing LX16A servo controller on port {}", port);

    // Initialize the controller with a timeout of 20ms
    let controller = LX16A::initialize(port, Duration::from_millis(20))?;

    // Create a map to store multiple servos
    let mut servo_map = HashMap::new();

    // Create shared state
    let state = Arc::new(AppState {
        servos: Mutex::new(servo_map),
        controller: Arc::new(Mutex::new(controller)),
        cache_ttl: Duration::from_millis(500_000_000), // Trust cache for 500ms
        force_read_interval: 500_000_000,              // Force real read every 10 requests
                                                       // force_read_interval: 10,               // Force real read every 10 requests
    });

    // Initialize available servos (IDs 1-12)
    // let servo_ids = [10, 11, 12];
    let servo_ids = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12];
    for id in servo_ids {
        if let Ok(()) = initialize_servo(&state, id) {
            println!("Servo ID {} initialized", id);
        } else {
            println!("Servo ID {} not connected or failed to initialize", id);
        }
    }

    match mode {
        "--bench" => {
            run_benchmark(&state, &servo_ids).await?;
        }
        _ => {
            // Build our application with routes
            let app = Router::new()
                // .route("/servos", get(handle_get_all_servos))
                .route("/servos", get(handle_get_all_servos_fast))
                .route("/servos/:id", get(handle_get_servo))
                .route("/servos/:id/position", post(handle_set_position))
                .with_state(state);

            // Start the server
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
        }
    }

    Ok(())
}

async fn run_benchmark(
    state: &Arc<AppState>,
    servo_ids: &[u8],
) -> Result<(), Box<dyn std::error::Error>> {
    println!("Running benchmark on servo angle reading (with interpolation)");
    println!("----------------------------------------");

    // const ITERATIONS: usize = 20;
    const ITERATIONS: usize = 1;

    for &id in servo_ids {
        let mut total_duration = Duration::new(0, 0);
        let mut successes = 0;
        let mut cache_hits = 0;

        // Check if the servo is connected
        let servo_map = state.servos.lock().unwrap();
        if !servo_map.contains_key(&id) {
            println!("Servo ID {} not found, skipping", id);
            continue;
        }
        drop(servo_map);

        println!("Benchmarking Servo ID: {}", id);

        for i in 1..=ITERATIONS {
            let start = Instant::now();

            match get_servo_position_info(state, id) {
                Ok((info, from_cache)) => {
                    let duration = start.elapsed();
                    total_duration += duration;
                    successes += 1;
                    if from_cache {
                        cache_hits += 1;
                        println!(
                            "  Iteration {}: {}° (took {:?}) [CACHED{}]",
                            i,
                            info.position,
                            duration,
                            if info.is_moving { " - MOVING" } else { "" }
                        );
                    } else {
                        println!(
                            "  Iteration {}: {}° (took {:?}) [REAL READ]",
                            i, info.position, duration
                        );
                    }
                }
                Err(e) => {
                    println!("  Iteration {}: Error: {:?}", i, e);
                }
            }

            // Short delay between readings
            tokio::time::sleep(Duration::from_millis(50)).await;
        }

        if successes > 0 {
            let avg_duration = total_duration / successes as u32;
            println!(
                "Servo ID {} - Average latency: {:?} over {} successful readings ({} from cache)",
                id, avg_duration, successes, cache_hits
            );
        } else {
            println!("Servo ID {} - No successful readings", id);
        }
        println!("----------------------------------------");
    }

    // Print cache statistics
    let servo_map = state.servos.lock().unwrap();
    for (&id, cached_servo) in servo_map.iter() {
        let total_reads = cached_servo.real_reads_count + cached_servo.cache_hits_count;
        if total_reads > 0 {
            let cache_hit_ratio =
                (cached_servo.cache_hits_count as f32 / total_reads as f32) * 100.0;
            println!(
                "Servo ID {} - Cache stats: {} real reads, {} cache hits ({}% hit rate)",
                id, cached_servo.real_reads_count, cached_servo.cache_hits_count, cache_hit_ratio
            );
        }
    }

    println!("Benchmark complete");
    Ok(())
}

fn initialize_servo(state: &Arc<AppState>, id: u8) -> LX16AResult<()> {
    let controller_clone = state.controller.lock().unwrap().clone();
    let servo = LX16A::new(id, controller_clone, false)?;

    // Try to get some basic info to verify the servo is responding
    let _id = servo.get_id(false)?;

    // Get the initial position
    let position = servo.get_physical_angle()?;

    // If we got here, the servo is responding, so add it to our map with initial cache
    let cached_servo = CachedServo {
        servo,
        cached_position: position,
        last_update: Instant::now(),
        real_reads_count: 1,
        cache_hits_count: 0,
        movement: None,
    };

    state.servos.lock().unwrap().insert(id, cached_servo);
    Ok(())
}

// Calculate the interpolated position based on movement parameters
fn calculate_interpolated_position(movement: &MovementState, now: Instant) -> (f32, bool) {
    let elapsed_ms = now.duration_since(movement.start_time).as_millis() as i32;

    // If we've exceeded the movement time, we're at the target position and no longer moving
    if elapsed_ms >= movement.movement_duration_ms {
        return (movement.target_position, false);
    }

    // We're still moving, calculate the interpolated position
    let progress = elapsed_ms as f32 / movement.movement_duration_ms as f32;
    let position_delta = movement.target_position - movement.start_position;
    let current_position = movement.start_position + (position_delta * progress);

    (current_position, true)
}

// Get servo info with full information (temperature, voltage, position)
fn get_servo_info(state: &AppState, id: u8) -> Result<ServoInfo, AppError> {
    let mut servo_map = state.servos.lock().unwrap();

    let cached_servo = servo_map
        .get_mut(&id)
        .ok_or_else(|| AppError::NotFound(format!("Servo with ID {} not found", id)))?;

    let id = match cached_servo.servo.get_id(false) {
        Ok(id) => id,
        Err(e) => return Err(AppError::ServoError(format!("Error getting ID: {:?}", e))),
    };

    let temperature = match cached_servo.servo.get_temp() {
        Ok(temp) => temp,
        Err(e) => {
            return Err(AppError::ServoError(format!(
                "Error getting temperature: {:?}",
                e
            )))
        }
    };

    let voltage = match cached_servo.servo.get_vin() {
        Ok(vin) => vin,
        Err(e) => {
            return Err(AppError::ServoError(format!(
                "Error getting voltage: {:?}",
                e
            )))
        }
    };

    // Check the movement status
    let now = Instant::now();
    let (position, is_moving) = if let Some(movement) = &cached_servo.movement {
        calculate_interpolated_position(movement, now)
    } else {
        // No movement in progress, always perform a real read for full info requests
        match cached_servo.servo.get_physical_angle() {
            Ok(angle) => {
                // Update the cache with the real position
                cached_servo.cached_position = angle;
                cached_servo.last_update = now;
                cached_servo.real_reads_count += 1;
                (angle, false)
            }
            Err(e) => {
                return Err(AppError::ServoError(format!(
                    "Error getting position: {:?}",
                    e
                )))
            }
        }
    };

    // If we were moving but now we're done, clear the movement state
    if !is_moving && cached_servo.movement.is_some() {
        cached_servo.movement = None;

        // Update the cache with the final position
        cached_servo.cached_position = position;
        cached_servo.last_update = now;
    }

    Ok(ServoInfo {
        id,
        temperature,
        voltage,
        position,
        is_moving,
    })
}

// Get servo position info as fast as possible (using cache when appropriate)
fn get_servo_position_info(
    state: &Arc<AppState>,
    id: u8,
) -> Result<(ServoPositionInfo, bool), AppError> {
    let mut servo_map = state.servos.lock().unwrap();

    let cached_servo = servo_map
        .get_mut(&id)
        .ok_or_else(|| AppError::NotFound(format!("Servo with ID {} not found", id)))?;

    let now = Instant::now();

    // // Check if we're currently in a movement
    // if let Some(movement) = &cached_servo.movement {
    //     let (position, is_moving) = calculate_interpolated_position(movement, now);

    //     // If we're done moving, update the cache and clear the movement
    //     if !is_moving {
    //         cached_servo.cached_position = position;
    //         cached_servo.last_update = now;
    //         cached_servo.movement = None;
    //     }

    //     cached_servo.cache_hits_count += 1;
    //     return Ok((
    //         ServoPositionInfo {
    //             id,
    //             position,
    //             is_moving,
    //         },
    //         true,
    //     ));
    // }

    // Determine if we should use the cache
    // let cache_age = now.duration_since(cached_servo.last_update);
    // let request_count = cached_servo.real_reads_count + cached_servo.cache_hits_count;

    // let use_cache = cache_age < state.cache_ttl && (request_count % state.force_read_interval != 0);
    let use_cache = false;

    if use_cache {
        // Use cached position
        cached_servo.cache_hits_count += 1;
        Ok((
            ServoPositionInfo {
                id,
                position: cached_servo.cached_position,
                is_moving: false,
            },
            true,
        ))
    } else {
        // Perform a real read
        match cached_servo.servo.get_physical_angle() {
            Ok(angle) => {
                // Update the cache
                cached_servo.cached_position = angle;
                cached_servo.last_update = now;
                cached_servo.real_reads_count += 1;

                Ok((
                    ServoPositionInfo {
                        id,
                        position: angle,
                        is_moving: false,
                    },
                    false,
                ))
            }
            Err(e) => Err(AppError::ServoError(format!(
                "Error getting position: {:?}",
                e
            ))),
        }
    }
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

async fn handle_get_all_servos_fast(
    State(state): State<Arc<AppState>>,
) -> Result<Json<AllServosPositionInfo>, AppError> {
    println!("Getting fast info for all servos");
    let start_time = Instant::now();
    let servo_map = state.servos.lock().unwrap();
    let servo_ids: Vec<u8> = servo_map.keys().cloned().collect();
    drop(servo_map); // Release the lock before the loop

    let mut servo_info_list = Vec::new();
    for id in servo_ids {
        match get_servo_position_info(&state, id) {
            Ok((info, _)) => servo_info_list.push(info),
            Err(e) => {
                eprintln!("Error getting info for servo {}: {:?}", id, e);
                // Continue with other servos even if one fails
            }
        }
    }

    // sort the servo info list by ID
    servo_info_list.sort_by_key(|info| info.id);
    let elapsed_time = start_time.elapsed();
    println!("Fast info retrieval took: {:?}", elapsed_time);
    Ok(Json(AllServosPositionInfo {
        servos: servo_info_list,
    }))
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
        match get_servo_info(&state, id) {
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
    let info = get_servo_info(&state, id)?;
    Ok(Json(info))
}

async fn handle_set_position(
    State(state): State<Arc<AppState>>,
    Path(id): Path<u8>,
    Json(body): Json<SetPositionRequest>,
) -> Result<StatusCode, AppError> {
    let target_position = body.position;
    let time_ms = body.time;

    let mut servo_map = state.servos.lock().unwrap();

    let cached_servo = servo_map
        .get_mut(&id)
        .ok_or_else(|| AppError::NotFound(format!("Servo with ID {} not found", id)))?;

    // Get the current position before issuing the movement command
    let current_position = if let Some(movement) = &cached_servo.movement {
        let (pos, _) = calculate_interpolated_position(movement, Instant::now());
        pos
    } else {
        cached_servo.cached_position
    };

    match cached_servo
        .servo
        .move_servo(target_position, time_ms, false, false)
    {
        Ok(_) => {
            println!(
                "Moving servo ID {} from {}° to {}° in {} ms",
                id, current_position, target_position, time_ms
            );

            // Set up the movement tracking
            if time_ms > 0 {
                cached_servo.movement = Some(MovementState {
                    start_position: current_position,
                    target_position,
                    start_time: Instant::now(),
                    movement_duration_ms: time_ms,
                });
            } else {
                // For immediate moves, just update the cached position
                cached_servo.cached_position = target_position;
                cached_servo.last_update = Instant::now();
                cached_servo.movement = None;
            }

            Ok(StatusCode::OK)
        }
        Err(e) => {
            let error_msg = format!("Error setting servo position: {:?}", e);
            eprintln!("{}", error_msg);
            Err(AppError::ServoError(error_msg))
        }
    }
}
