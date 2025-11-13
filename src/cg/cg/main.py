import rclpy
import threading
import os
import random
import sys
import argparse
from ament_index_python.packages import get_package_share_directory

from .Utils.Csv import load_from_csv
from .Game import Game
from .Editor import Editor


def game():
    parser = argparse.ArgumentParser(description='Culling Games Maze Runner')
    parser.add_argument('--map', type=str, help='Name of the map file to load from the maps directory (e.g., "test.csv")')
    args, unknown = parser.parse_known_args()

    rclpy.init(args=[arg for arg in sys.argv if arg not in ('-h', '--help')])
    
    share_dir = get_package_share_directory('cg')
    maps_dir = os.path.join(share_dir, 'maps')
    
    map_name = None
    if args.map:
        map_name = args.map
        if not map_name.endswith('.csv'):
            map_name += '.csv'
    else:
        available_maps = [f for f in os.listdir(maps_dir) if f.endswith('.csv')]
        if not available_maps:
            raise FileNotFoundError("No map files (.csv) found in the maps directory.")
        map_name = random.choice(available_maps)

    map_path = os.path.join(maps_dir, map_name)
    
    if not os.path.exists(map_path):
        print(f"Error: Map file not found at {map_path}", file=sys.stderr)
        return

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
