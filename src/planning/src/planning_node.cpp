#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose2D.h>
#include <queue>
#include <vector>
#include <cmath>

// Глобальные переменные для хранения информации о карте
nav_msgs::OccupancyGrid map_data;

// Вспомогательная структура для представления узла в графе
struct Node {
  int x;
  int y;
  float g; // Стоимость пути от начальной вершины до текущей
  float h; // Эвристическая оценка стоимости пути от текущей вершины до цели
  float f; // Оценка общей стоимости пути через текущую вершину
  Node* parent; // Родительский узел в пути

  Node(int x, int y, float g, float h, Node* parent) : x(x), y(y), g(g), h(h), parent(parent) {
    f = g + h;
  }

  // Оператор сравнения для приоритетной очереди
  bool operator<(const Node& other) const {
    return f > other.f; // Меньшее значение f имеет более высокий приоритет
  }
};

// Функция обратного вызова для получения карты
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  map_data = *msg;
}

// Функция для вычисления эвристической оценки стоимости пути от текущей вершины до цели
float heuristic(int x1, int y1, int x2, int y2) {
  return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

// Функция для построения пути с использованием алгоритма A*
nav_msgs::Path buildPath(const geometry_msgs::Pose2D& start_pose, const geometry_msgs::Pose2D& finish_pose) {
  nav_msgs::Path planned_path;
  planned_path.header.frame_id = "map";

  // Инициализация начального и конечного узлов
  int start_x = (start_pose.x - map_data.info.origin.position.x) / map_data.info.resolution;
  int start_y = (start_pose.y - map_data.info.origin.position.y) / map_data.info.resolution;
  int finish_x = (finish_pose.x - map_data.info.origin.position.x) / map_data.info.resolution;
  int finish_y = (finish_pose.y - map_data.info.origin.position.y) / map_data.info.resolution;

  std::priority_queue<Node> open_set;
  std::vector<std::vector<bool>> closed_set(map_data.info.width, std::vector<bool>(map_data.info.height, false));

  // Вставка начального узла в приоритетную очередь
  open_set.push(Node(start_x, start_y, 0, heuristic(start_x, start_y, finish_x, finish_y), nullptr));

  // Просмотр соседей текущего узла
  while (!open_set.empty()) {
    // Извлечение узла с наименьшим f из очереди
    Node current = open_set.top();
    open_set.pop();

    if (current.x == finish_x && current.y == finish_y) {
      // Построение пути, обратным ходом от цели к началу
      while (current.parent != nullptr) {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = current.x * map_data.info.resolution + map_data.info.origin.position.x;
        pose.pose.position.y = current.y * map_data.info.resolution + map_data.info.origin.position.y;
        planned_path.poses.push_back(pose);

        current = *(current.parent);
      }

      // Добавление начальной позиции
      geometry_msgs::PoseStamped start_pose_stamped;
      start_pose_stamped.pose.position.x = start_pose.x;
      start_pose_stamped.pose.position.y = start_pose.y;
      planned_path.poses.push_back(start_pose_stamped);

      // Переворачиваем путь, чтобы он был в порядке от начала к концу
      std::reverse(planned_path.poses.begin(), planned_path.poses.end());

      return planned_path;
    }

    closed_set[current.x][current.y] = true;

    // Просмотр соседей текущего узла
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        int new_x = current.x + dx;
        int new_y = current.y + dy;

        // Проверка находится ли сосед в пределах карты
        if (new_x < 0 || new_x >= map_data.info.width || new_y < 0 || new_y >= map_data.info.height)
          continue;

        // Проверка проходимости соседа
        int index = new_y * map_data.info.width + new_x;
        if (map_data.data[index] != 0)
          continue;

        // Проверка, не был ли сосед уже посещен
        if (closed_set[new_x][new_y])
          continue;

        // Расчет стоимости пути до соседа
        float tentative_g = current.g + map_data.info.resolution; // шаг робота

        // Проверка, не был ли достигнут более короткий путь к соседу ранее
        bool in_open_set = false;
        std::priority_queue<Node> open_set_copy = open_set; // Копия для итерации по элементам
        while (!open_set_copy.empty()) {
          Node node = open_set_copy.top();
          open_set_copy.pop();
          if (node.x == new_x && node.y == new_y && node.f < tentative_g) {
            in_open_set = true;
            break;
          }
        }
        if (in_open_set)
          continue;

        // Добавление соседа в открытый список
        open_set.push(Node(new_x, new_y, tentative_g, heuristic(new_x, new_y, finish_x, finish_y), new Node(current)));
      }
    }
  }

  // Если путь не найден, возвращаем пустой путь
  return planned_path;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_a_node");
  ros::NodeHandle nh;

  // Подписка на сообщения о карте от первого узла
  ros::Subscriber map_sub = nh.subscribe("/tr_map", 1, mapCallback);

  // Создание публикатора для публикации пути
  ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/planned_path", 1);

  // Определение начальной и конечной позиции
  geometry_msgs::Pose2D start_pose, finish_pose;
  start_pose.x = 0;
  start_pose.y = 0;
  finish_pose.x = 1.5;
  finish_pose.y = 1.5;

  ros::Rate loop_rate(10);

  while (ros::ok()) {
    // Построение пути
    nav_msgs::Path planned_path = buildPath(start_pose, finish_pose);

    // Публикация пути
    path_pub.publish(planned_path);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
