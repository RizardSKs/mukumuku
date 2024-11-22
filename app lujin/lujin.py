from flask import Flask, request, jsonify
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2
from flask_cors import CORS
import os

app = Flask(__name__)
CORS(app)

def calculate_optimal_waypoints(waypoints):
    num_waypoints = len(waypoints)

    # 创建 Routing 模型
    manager = pywrapcp.RoutingIndexManager(num_waypoints, 1, 0)
    routing = pywrapcp.RoutingModel(manager)

    # 创建距离回调函数
    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return distance_between_points(waypoints[from_node], waypoints[to_node])

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # 设置距离约束
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # 设置搜索参数
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )

    # 解决问题
    solution = routing.SolveWithParameters(search_parameters)

    # 提取路径
    sorted_indices = []
    visited_nodes = set()
    index = routing.Start(0)
    while not routing.IsEnd(index):
        node = manager.IndexToNode(index)
        if node not in visited_nodes:
            sorted_indices.append(node)
            visited_nodes.add(node)
        index = solution.Value(routing.NextVar(index))
    last_node = manager.IndexToNode(index)
    if last_node != 0:  # 确保最后一个节点不是起点
        sorted_indices.append(last_node)

    # 根据排序的索引重排途径点
    sorted_waypoints = [waypoints[i] for i in sorted_indices]

    return sorted_waypoints



def distance_between_points(point1, point2):
    # 计算两个点之间的欧氏距离
    return ((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)**0.5

@app.route('/api/optimize-waypoints', methods=['POST'])
def optimize_waypoints():
    data = request.json
    waypoints = data.get('waypoints', [])
    sorted_waypoints = calculate_optimal_waypoints(waypoints)
    return jsonify({'waypoints': sorted_waypoints})

debug_mode = os.getenv("DEBUG", "false").lower() == "true"

if __name__ == '__main__':
    port = int(os.getenv("PORT", 5000))
    app.run(host='0.0.0.0', port=port, debug=debug_mode)
