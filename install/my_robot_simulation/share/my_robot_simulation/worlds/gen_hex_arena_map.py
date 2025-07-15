import xml.etree.ElementTree as ET
import numpy as np
from PIL import Image

# 地图参数
dim = 100  # 地图边长（米）
resolution = 0.05  # 米/像素
size = int(dim / resolution)  # 像素数
origin = (-50, -50, 0)

# 读取world文件
world_path = 'my_hex_arena.world'
tree = ET.parse(world_path)
root = tree.getroot()

# SDF 1.7命名空间
ns = ''
if root.tag.startswith('{'):
    ns = root.tag.split('}')[0] + '}'

def parse_pose(pose_str):
    vals = [float(x) for x in pose_str.strip().split()]
    if len(vals) == 3:
        return vals + [0, 0, 0]
    return vals

def world_to_pixel(x, y):
    px = int((x - origin[0]) / resolution)
    py = size - int((y - origin[1]) / resolution)  # y轴反向
    return px, py

# 初始化全白地图
img = np.ones((size, size), dtype=np.uint8) * 254

for model in root.findall(f'.//{ns}model'):
    pose_elem = model.find(f'{ns}pose')
    pose = [0, 0, 0, 0, 0, 0]
    if pose_elem is not None:
        pose = parse_pose(pose_elem.text)
    link = model.find(f'{ns}link')
    if link is None:
        continue
    # box
    box = link.find(f'.//{ns}box')
    if box is not None:
        size_elem = box.find(f'{ns}size')
        if size_elem is not None and size_elem.text is not None:
            sx, sy, sz = [float(x) for x in size_elem.text.strip().split()]
            # 计算四个角点（不考虑旋转时）
            cx, cy = pose[0], pose[1]
            theta = pose[5]  # 只考虑z轴旋转
            # 旋转和平移
            num_x = max(2, int(sx / resolution))
            num_y = max(2, int(sy / resolution))
            for dx in np.linspace(-sx/2, sx/2, num_x):
                for dy in np.linspace(-sy/2, sy/2, num_y):
                    x = cx + dx * np.cos(theta) - dy * np.sin(theta)
                    y = cy + dx * np.sin(theta) + dy * np.cos(theta)
                    px, py = world_to_pixel(x, y)
                    if 0 <= px < size and 0 <= py < size:
                        img[py, px] = 0
    # cylinder
    cyl = link.find(f'.//{ns}cylinder')
    if cyl is not None:
        radius_elem = cyl.find(f'{ns}radius')
        if radius_elem is not None and radius_elem.text is not None:
            r = float(radius_elem.text)
            cx, cy = pose[0], pose[1]
            for dx in np.arange(-r, r, resolution/2):
                for dy in np.arange(-r, r, resolution/2):
                    if dx**2 + dy**2 <= r**2:
                        x = cx + dx
                        y = cy + dy
                        px, py = world_to_pixel(x, y)
                        if 0 <= px < size and 0 <= py < size:
                            img[py, px] = 0

# 保存pgm
im = Image.fromarray(img)
im.save('my_hex_arena.pgm')

# 保存yaml
with open('my_hex_arena.yaml', 'w') as f:
    f.write(f"""image: my_hex_arena.pgm
resolution: {resolution}
origin: [{origin[0]}, {origin[1]}, {origin[2]}]
negate: 0
occupied_thresh: 0.5
free_thresh: 0.2
""")

print('地图已生成：my_hex_arena.pgm, my_hex_arena.yaml') 