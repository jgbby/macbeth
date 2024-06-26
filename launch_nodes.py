import yaml
from ament_index_python.packages import get_package_share_directory
from crazyflie_py import Crazyswarm
import rclpy
import threading

# Inject Imports Here:

def launch(nodes):
    executor = rclpy.executors.MultiThreadedExecutor()

    for node in nodes:
        executor.add_node(node)
    
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        while rclpy.ok():
            pass
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
    thread.join()
    

def main():
    # node_config_file = get_package_share_directory('crazyflie'), 'config', 'crazyflies.yaml'
    
    # Initialize swarm
    swarm = Crazyswarm()
    with open("cfs_ordering.yaml") as f:
        ordering = yaml.safe_load(f) #TODO!!!
        order = ordering['cfs']
    
    crazyflies = [swarm.allcfs.crazyfliesById[int(k)] for k in order]
    counter = 0
    nodes = []
    cfs = []

    #   -----------Insert Nodes Here----------- 
    import group4_node
    cfs = []
    cfs.append(crazyflies[0])
    nodes.append(group4_node.worker_node(cfs, len(nodes), 5))


    import group5_node
    cfs = []
    cfs.append(crazyflies[1])
    nodes.append(group5_node.worker_node(cfs, len(nodes), 6))


    import group6_node
    cfs = []
    cfs.append(crazyflies[2])
    nodes.append(group6_node.worker_node(cfs, len(nodes), 7))


    import group7_node
    cfs = []
    cfs.append(crazyflies[3])
    nodes.append(group7_node.worker_node(cfs, len(nodes), 8))


    import group8_node
    cfs = []
    cfs.append(crazyflies[4])
    nodes.append(group8_node.worker_node(cfs, len(nodes), 9))


    import group9_node
    cfs = []
    cfs.append(crazyflies[5])
    nodes.append(group9_node.worker_node(cfs, len(nodes), 10))


    import group10_node
    cfs = []
    cfs.append(crazyflies[6])
    nodes.append(group10_node.worker_node(cfs, len(nodes), 11))


    # Launch all nodes
    return launch(nodes)

if __name__ == '__main__':
    main()
