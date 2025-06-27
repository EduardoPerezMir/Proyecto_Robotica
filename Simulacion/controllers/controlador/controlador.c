#include <webots/distance_sensor.h>
#include <webots/lidar.h>
#include <webots/gps.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#define TIME_STEP 64
#define GRID_SIZE 80  
#define CELL_SIZE 0.5
#define MAX_PATH_LEN 100
#define MAX_OPEN_NODES 1000
#define MAX_WHEELS 4

#define SPEED 10.0
#define WAYPOINT_THRESHOLD 0.3  // distancia para considerar waypoint alcanzado
#define OBSTACLE_THRESHOLD_LIDAR 0.15
#define OBSTACLE_THRESHOLD_DS 100.0

typedef struct {
  int x, y;
} Point;

typedef struct {
  int x, y;
  int g, h, f;
  int parent_index;
} Node;

// Convierte coordenadas mundiales (metros) a índices en la grilla
Point world_to_grid(double x, double y) {
  Point p;
  p.x = (int)((x + (GRID_SIZE * CELL_SIZE) / 2.0) / CELL_SIZE);
  p.y = (int)((y + (GRID_SIZE * CELL_SIZE) / 2.0) / CELL_SIZE);
  return p;
}

// Heurística Manhattan para A*
int heuristic(int x1, int y1, int x2, int y2) {
  return abs(x1 - x2) + abs(y1 - y2);
}

// Algoritmo A* para planificar ruta en la grilla
int plan_path(int grid[GRID_SIZE][GRID_SIZE], Point start, Point goal, Point path[], int max_path_len) {
  Node open_list[MAX_OPEN_NODES];
  int open_count = 0;

  Node closed_list[GRID_SIZE * GRID_SIZE];
  int closed_count = 0;

  Node start_node = {start.x, start.y, 0, heuristic(start.x, start.y, goal.x, goal.y), 0, -1};
  start_node.f = start_node.g + start_node.h;
  open_list[open_count++] = start_node;

  while (open_count > 0) {
    // Buscar nodo con menor f
    int best_index = 0;
    for (int i = 1; i < open_count; i++) {
      if (open_list[i].f < open_list[best_index].f)
        best_index = i;
    }

    Node current = open_list[best_index];
    // Eliminar current de open_list
    for (int i = best_index; i < open_count - 1; i++)
      open_list[i] = open_list[i + 1];
    open_count--;
    closed_list[closed_count++] = current;

    // Si llegó al destino, reconstruir ruta
    if (current.x == goal.x && current.y == goal.y) {
      int length = 0;
      Node n = current;
      while (n.parent_index != -1 && length < max_path_len) {
        path[length++] = (Point){n.x, n.y};
        n = closed_list[n.parent_index];
      }
      path[length++] = (Point){start.x, start.y};

      // Invertir path para que vaya de start a goal
      for (int i = 0; i < length / 2; i++) {
        Point temp = path[i];
        path[i] = path[length - i - 1];
        path[length - i - 1] = temp;
      }
      return length;
    }

    // Explorar vecinos (4 direcciones)
    const int dx[4] = {0, 1, 0, -1};
    const int dy[4] = {1, 0, -1, 0};
    for (int d = 0; d < 4; d++) {
      int nx = current.x + dx[d];
      int ny = current.y + dy[d];
      if (nx < 0 || ny < 0 || nx >= GRID_SIZE || ny >= GRID_SIZE)
        continue;
      if (grid[nx][ny] == 1) // obstáculo
        continue;

      // Revisar si ya está cerrado
      bool in_closed = false;
      for (int i = 0; i < closed_count; i++) {
        if (closed_list[i].x == nx && closed_list[i].y == ny) {
          in_closed = true;
          break;
        }
      }
      if (in_closed) continue;

      int g = current.g + 1;
      int h = heuristic(nx, ny, goal.x, goal.y);
      int f = g + h;

      // Buscar en open list
      int in_open = -1;
      for (int i = 0; i < open_count; i++) {
        if (open_list[i].x == nx && open_list[i].y == ny) {
          in_open = i;
          break;
        }
      }

      if (in_open != -1) {
        if (f < open_list[in_open].f) {
          open_list[in_open].g = g;
          open_list[in_open].h = h;
          open_list[in_open].f = f;
          open_list[in_open].parent_index = closed_count - 1;
        }
      } else if (open_count < MAX_OPEN_NODES) {
        Node neighbor = {nx, ny, g, h, f, closed_count - 1};
        open_list[open_count++] = neighbor;
      }
    }
  }
  // No encontró ruta
  return 0;
}

// Función para actualizar mapa con datos LIDAR
void update_map_with_lidar(int grid[GRID_SIZE][GRID_SIZE], const float *ranges, int resolution, double fov, double robot_x, double robot_y) {
  for (int i = 0; i < resolution; i++) {
    double angle = -fov / 2 + i * (fov / resolution);
    double dist = ranges[i];

    if (isinf(dist) || isnan(dist)) continue;

    // Si distancia menor a un umbral, marcar obstáculo en la grilla
    if (dist < 1.0) {
      double obs_x = robot_x + dist * cos(angle);
      double obs_y = robot_y + dist * sin(angle);

      int cell_x = (int)((obs_x + GRID_SIZE * CELL_SIZE / 2) / CELL_SIZE);
      int cell_y = (int)((obs_y + GRID_SIZE * CELL_SIZE / 2) / CELL_SIZE);

      if (cell_x >= 0 && cell_x < GRID_SIZE && cell_y >= 0 && cell_y < GRID_SIZE)
        grid[cell_x][cell_y] = 1;  // Ocupado
    }
  }
}

// Control diferencial para mover hacia waypoint
void move_towards_waypoint(WbDeviceTag wheels[], double robot_x, double robot_y, Point waypoint, double *left_speed, double *right_speed) {
  double target_x = (waypoint.x * CELL_SIZE) - (GRID_SIZE * CELL_SIZE / 2.0);
  double target_y = (waypoint.y * CELL_SIZE) - (GRID_SIZE * CELL_SIZE / 2.0);

  double dx = target_x - robot_x;
  double dy = target_y - robot_y;
  double distance = sqrt(dx*dx + dy*dy);

  double target_angle = atan2(dy, dx);

  // Aquí no tenemos orientación real del robot, asumimos control simple
  // Para mejor control, se necesitaría un giroscopio o IMU para obtener yaw
  double angle_diff = target_angle; // simplificado

  if (angle_diff > 0.1) {
    *left_speed = SPEED;
    *right_speed = SPEED * 0.3;
  } else if (angle_diff < -0.1) {
    *left_speed = SPEED * 0.3;
    *right_speed = SPEED;
  } else {
    *left_speed = SPEED;
    *right_speed = SPEED;
  }
}

int main() {
  wb_robot_init();

  // Inicializar motores
  WbDeviceTag wheels[MAX_WHEELS];
  char wheels_names[MAX_WHEELS][8] = {"wheel1", "wheel2", "wheel3", "wheel4"};
  for (int i = 0; i < MAX_WHEELS; i++) {
    wheels[i] = wb_robot_get_device(wheels_names[i]);
    wb_motor_set_position(wheels[i], INFINITY);
  }

  // Inicializar sensores de distancia
  WbDeviceTag ds[2];
  char ds_names[2][10] = {"ds_left", "ds_right"};
  for (int i = 0; i < 2; i++) {
    ds[i] = wb_robot_get_device(ds_names[i]);
    wb_distance_sensor_enable(ds[i], TIME_STEP);
  }

  // Inicializar LIDAR
  WbDeviceTag lidar = wb_robot_get_device("lidar");
  wb_lidar_enable(lidar, TIME_STEP);
  wb_lidar_enable_point_cloud(lidar);

  // Inicializar GPS
  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, TIME_STEP);

  int grid[GRID_SIZE][GRID_SIZE] = {0};
  Point path[MAX_PATH_LEN];
  int path_length = 0;
  int current_waypoint = 0;

  while (wb_robot_step(TIME_STEP) != -1) {
    // Leer sensores distancia
    bool ds_detect_near = false;
    for (int i = 0; i < 2; i++) {
      double val = wb_distance_sensor_get_value(ds[i]);
      if (val < OBSTACLE_THRESHOLD_DS) ds_detect_near = true;
    }

    // Leer posición GPS
    const double *pose = wb_gps_get_values(gps);
    double robot_x = pose[0];
    double robot_y = pose[2];

    // Leer LIDAR y actualizar mapa
    const float *ranges = wb_lidar_get_range_image(lidar);
    int resolution = wb_lidar_get_horizontal_resolution(lidar);
    double fov = wb_lidar_get_fov(lidar);
    update_map_with_lidar(grid, ranges, resolution, fov, robot_x, robot_y);

    // Detectar obstáculos muy cerca con LIDAR
    bool lidar_detect_near = false;
    for (int i = 0; i < resolution; i++) {
      if (!isinf(ranges[i]) && ranges[i] < OBSTACLE_THRESHOLD_LIDAR) {
        lidar_detect_near = true;
        break;
      }
    }

    // Punto objetivo fijo (puede ser parametrizado)
    Point goal = world_to_grid(-4.649370, 0.06975);
    Point start = world_to_grid(robot_x, robot_y);

    // Planificar ruta si el camino actual no es válido o se terminó
    if (path_length == 0 || current_waypoint >= path_length) {
      path_length = plan_path(grid, start, goal, path, MAX_PATH_LEN);
      current_waypoint = 0;
      printf("Planificando ruta, pasos: %d\n", path_length);
      if (path_length == 0) {
        printf("No se encontró ruta válida\n");
      }
    }

    double left_speed = 0.0, right_speed = 0.0;

    // Comportamiento evasivo si obstáculo está muy cerca
    if (lidar_detect_near || ds_detect_near) {
      printf("Obstáculo cercano, maniobra evasiva\n");
      left_speed = SPEED;
      right_speed = -SPEED;
    } else if (path_length > 0 && current_waypoint < path_length) {
      Point waypoint = path[current_waypoint];
      // Calcular distancia al waypoint
      double wx = (waypoint.x * CELL_SIZE) - (GRID_SIZE * CELL_SIZE / 2.0);
      double wy = (waypoint.y * CELL_SIZE) - (GRID_SIZE * CELL_SIZE / 2.0);
      double dist_wp = sqrt(pow(wx - robot_x, 2) + pow(wy - robot_y, 2));

      if (dist_wp < WAYPOINT_THRESHOLD) {
        current_waypoint++;
        printf("Waypoint %d/%d alcanzado\n", current_waypoint, path_length);
      } else {
        move_towards_waypoint(wheels, robot_x, robot_y, waypoint, &left_speed, &right_speed);
      }
    } else {
      printf("Destino alcanzado o sin ruta\n");
      left_speed = 0;
      right_speed = 0;
    }

    // Aplicar velocidades a motores (4 ruedas, asumiendo cinemática diferencial)
    wb_motor_set_velocity(wheels[0], left_speed);
    wb_motor_set_velocity(wheels[1], right_speed);
    wb_motor_set_velocity(wheels[2], left_speed);
    wb_motor_set_velocity(wheels[3], right_speed);

    fflush(stdout);
  }

  wb_robot_cleanup();
  return 0;
}
