import rclpy
import threading
import os
import random
from ament_index_python.packages import get_package_share_directory

from .Utils.Csv import load_from_csv
from .Game import Game
from .Editor import Editor


def game():
    rclpy.init()
    share_dir = get_package_share_directory('cg')
    maps_dir = os.path.join(share_dir, 'maps')
    available_maps = [f for f in os.listdir(maps_dir) if f.endswith('.csv')]
    if not available_maps:
        raise FileNotFoundError("No map files (.csv) found in the maps directory.")
    
    map_name = random.choice(available_maps)
    map_path = os.path.join(maps_dir, map_name)
    
    game = Game(map_path)
    thread = threading.Thread(target=rclpy.spin, args=(game,))
    thread.start()

    try:
        game.run()
    except KeyboardInterrupt:
        game.get_logger().info('KeyboardInterrupt received, shutting down.')
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        thread.join()

def editor():
    share_dir = get_package_share_directory('cg')
    map_path = os.path.join(share_dir, 'maps', 'default.csv')
    editor = Editor(load_from_csv(map_path))
    editor.run()
