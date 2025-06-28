#!/usr/bin/env python3
# Copyright 2024 wb
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Map management utility for robot navigation.

This script provides tools to list, copy, delete, and manage map files
created during SLAM sessions.
"""

import os
import sys
import shutil
import argparse
from pathlib import Path
from datetime import datetime
from ament_index_python.packages import get_package_share_directory


class MapManager:
    """Utility class for managing robot maps."""
    
    def __init__(self):
        """Initialize map manager with package paths."""
        try:
            nav_pkg = get_package_share_directory('my_robot_navigation')
            self.maps_dir = Path(nav_pkg) / 'maps'
        except Exception:
            # Fallback to source directory if package not installed
            script_dir = Path(__file__).parent
            self.maps_dir = script_dir.parent / 'maps'
        
        # Create maps directory if it doesn't exist
        self.maps_dir.mkdir(exist_ok=True)
    
    def list_maps(self):
        """List all available maps."""
        yaml_files = list(self.maps_dir.glob('*.yaml'))
        
        if not yaml_files:
            print("📁 No maps found in the maps directory.")
            print(f"   Maps directory: {self.maps_dir}")
            return
        
        print("🗺️  Available maps:")
        print("-" * 50)
        
        for yaml_file in sorted(yaml_files):
            map_name = yaml_file.stem
            pgm_file = yaml_file.with_suffix('.pgm')
            
            # Get file info
            yaml_size = yaml_file.stat().st_size if yaml_file.exists() else 0
            pgm_size = pgm_file.stat().st_size if pgm_file.exists() else 0
            modified_time = datetime.fromtimestamp(yaml_file.stat().st_mtime)
            
            status = "✅ Complete" if pgm_file.exists() else "⚠️  Missing .pgm"
            
            print(f"📍 {map_name}")
            print(f"   Status: {status}")
            print(f"   Modified: {modified_time.strftime('%Y-%m-%d %H:%M:%S')}")
            print(f"   Size: YAML={yaml_size}B, PGM={pgm_size}B")
            print()
    
    def copy_map(self, source_name, target_name):
        """Copy a map to a new name."""
        source_yaml = self.maps_dir / f"{source_name}.yaml"
        source_pgm = self.maps_dir / f"{source_name}.pgm"
        target_yaml = self.maps_dir / f"{target_name}.yaml"
        target_pgm = self.maps_dir / f"{target_name}.pgm"
        
        if not source_yaml.exists():
            print(f"❌ Source map '{source_name}' not found!")
            return False
        
        try:
            # Copy YAML file
            shutil.copy2(source_yaml, target_yaml)
            
            # Copy PGM file if it exists
            if source_pgm.exists():
                shutil.copy2(source_pgm, target_pgm)
                
                # Update the image filename in YAML
                self._update_yaml_image_path(target_yaml, f"{target_name}.pgm")
            
            print(f"✅ Map copied: '{source_name}' → '{target_name}'")
            return True
            
        except Exception as e:
            print(f"❌ Failed to copy map: {e}")
            return False
    
    def delete_map(self, map_name, confirm=True):
        """Delete a map and its associated files."""
        yaml_file = self.maps_dir / f"{map_name}.yaml"
        pgm_file = self.maps_dir / f"{map_name}.pgm"
        
        if not yaml_file.exists():
            print(f"❌ Map '{map_name}' not found!")
            return False
        
        if confirm:
            response = input(f"⚠️  Are you sure you want to delete '{map_name}'? (y/N): ")
            if response.lower() != 'y':
                print("🚫 Deletion cancelled.")
                return False
        
        try:
            # Delete YAML file
            yaml_file.unlink()
            print(f"🗑️  Deleted: {yaml_file.name}")
            
            # Delete PGM file if it exists
            if pgm_file.exists():
                pgm_file.unlink()
                print(f"🗑️  Deleted: {pgm_file.name}")
            
            print(f"✅ Map '{map_name}' deleted successfully!")
            return True
            
        except Exception as e:
            print(f"❌ Failed to delete map: {e}")
            return False
    
    def set_default_map(self, map_name):
        """Set a map as the default for navigation."""
        source_yaml = self.maps_dir / f"{map_name}.yaml"
        source_pgm = self.maps_dir / f"{map_name}.pgm"
        default_yaml = self.maps_dir / "my_robot_map.yaml"
        default_pgm = self.maps_dir / "my_robot_map.pgm"
        
        if not source_yaml.exists():
            print(f"❌ Map '{map_name}' not found!")
            return False
        
        try:
            # Copy to default names
            shutil.copy2(source_yaml, default_yaml)
            
            if source_pgm.exists():
                shutil.copy2(source_pgm, default_pgm)
                # Update image path in default YAML
                self._update_yaml_image_path(default_yaml, "my_robot_map.pgm")
            
            print(f"✅ '{map_name}' set as default map for navigation!")
            return True
            
        except Exception as e:
            print(f"❌ Failed to set default map: {e}")
            return False
    
    def _update_yaml_image_path(self, yaml_file, new_image_name):
        """Update the image path in a YAML map file."""
        try:
            # Read and modify YAML content
            with open(yaml_file, 'r', encoding='utf-8') as f:
                lines = f.readlines()
            
            # Update image line in-place
            for i, line in enumerate(lines):
                if line.strip().startswith('image:'):
                    lines[i] = f"image: {new_image_name}\n"
                    break
            
            # Write back atomically
            with open(yaml_file, 'w', encoding='utf-8') as f:
                f.writelines(lines)
                
        except Exception as e:
            print(f"⚠️  Warning: Failed to update image path in YAML: {e}")


def main():
    """Main function for map management script."""
    parser = argparse.ArgumentParser(description='Manage robot navigation maps')
    subparsers = parser.add_subparsers(dest='command', help='Available commands')
    
    # List command
    subparsers.add_parser('list', help='List all available maps')
    
    # Copy command
    copy_parser = subparsers.add_parser('copy', help='Copy a map to a new name')
    copy_parser.add_argument('source', help='Source map name')
    copy_parser.add_argument('target', help='Target map name')
    
    # Delete command
    delete_parser = subparsers.add_parser('delete', help='Delete a map')
    delete_parser.add_argument('map_name', help='Name of map to delete')
    delete_parser.add_argument('--force', action='store_true', help='Skip confirmation')
    
    # Default command
    default_parser = subparsers.add_parser('default', help='Set a map as default for navigation')
    default_parser.add_argument('map_name', help='Name of map to set as default')
    
    args = parser.parse_args()
    
    if not args.command:
        parser.print_help()
        return
    
    manager = MapManager()
    
    if args.command == 'list':
        manager.list_maps()
    elif args.command == 'copy':
        manager.copy_map(args.source, args.target)
    elif args.command == 'delete':
        manager.delete_map(args.map_name, confirm=not args.force)
    elif args.command == 'default':
        manager.set_default_map(args.map_name)


if __name__ == '__main__':
    main()