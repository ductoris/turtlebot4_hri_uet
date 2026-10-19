#!/usr/bin/env python3
"""Sinh đồng thời world Ignition Gazebo (home_room.sdf) và bản đồ Nav2
(home_room.pgm / home_room.yaml) từ CÙNG MỘT bảng layout.

=> Bản đồ luôn khớp với thế giới mô phỏng (không cần chạy SLAM nếu lười).
Muốn sửa phòng: chỉnh danh sách BOXES / FUEL_OBJECTS rồi chạy lại:

    python3 scripts/generate_world_and_map.py

Hệ toạ độ: world == map (robot spawn tại (0,0), yaw=0).
"""
import os
import numpy as np
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
WORLD_DIR = os.path.join(PKG, 'worlds')
MAP_DIR = os.path.join(PKG, 'maps')

# ---------------------------------------------------------------- Layout ----
ROOM_X = 4.0          # phòng rộng 8 m  (x: -4 .. 4)
ROOM_Y = 3.0          # phòng dài 6 m   (y: -3 .. 3)
WALL_T = 0.2          # độ dày tường
WALL_H = 2.0

# (name, cx, cy, sx, sy, height, (r,g,b))  -- vật thể hình hộp, footprint chính xác
BOXES = [
    ('table',      1.8,  1.0, 1.4, 0.8, 0.75, (0.55, 0.35, 0.20)),
    ('shelf_west', -3.75, 1.2, 0.4, 2.0, 1.80, (0.45, 0.30, 0.15)),
    ('shelf_north', 0.5, 2.75, 1.6, 0.4, 1.40, (0.45, 0.30, 0.15)),
    ('sofa',       -2.4, -2.5, 1.8, 0.8, 0.50, (0.25, 0.35, 0.55)),
    ('cabinet',     3.7, -1.5, 0.4, 1.2, 1.00, (0.70, 0.70, 0.65)),
]

# Vật thể tải từ Fuel (để YOLO nhận diện): (name, uri, x, y, yaw, bán kính footprint để vẽ lên map)
FUEL = 'https://fuel.gazebosim.org/1.0/OpenRobotics/models/'
FUEL_OBJECTS = [
    ('person_1',   FUEL + 'Standing person', -1.2, 1.0, -1.57, 0.25),
    ('chair_1',    FUEL + 'Chair',            1.8, 0.2,  1.57, 0.25),
    ('chair_2',    FUEL + 'Chair',           -3.0, -0.5, 0.00, 0.25),
    ('hydrant_1',  FUEL + 'Fire hydrant',     3.4, 2.4,  0.00, 0.20),
]

# ------------------------------------------------------------ Map params ----
RES = 0.05
MAP_X0, MAP_Y0 = -5.0, -4.0     # origin (góc dưới-trái)
MAP_W, MAP_H = 10.0, 8.0

WALLS = [  # (name, cx, cy, sx, sy)
    ('wall_n', 0.0,  ROOM_Y + WALL_T / 2, 2 * ROOM_X + 2 * WALL_T, WALL_T),
    ('wall_s', 0.0, -ROOM_Y - WALL_T / 2, 2 * ROOM_X + 2 * WALL_T, WALL_T),
    ('wall_e',  ROOM_X + WALL_T / 2, 0.0, WALL_T, 2 * ROOM_Y),
    ('wall_w', -ROOM_X - WALL_T / 2, 0.0, WALL_T, 2 * ROOM_Y),
]


def box_link(name, cx, cy, sx, sy, h, rgb):
    r, g, b = rgb
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{cx} {cy} {h / 2} 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>{sx} {sy} {h}</size></box></geometry></collision>
        <visual name="visual">
          <geometry><box><size>{sx} {sy} {h}</size></box></geometry>
          <material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse></material>
        </visual>
      </link>
    </model>"""


def build_sdf():
    parts = ["""<?xml version="1.0"?>
<!-- Sinh tự động bởi scripts/generate_world_and_map.py - ĐỪNG sửa tay, hãy sửa script.
     Tên world BẮT BUỘC là "warehouse": bridge của turtlebot4_ignition_bringup dùng tên này
     trong các topic /world/warehouse/model/... (nếu đổi, camera/lidar sẽ không lên ROS). -->
<sdf version="1.6">
  <world name="warehouse">
    <physics name="1ms" type="ignored">
      <max_step_size>0.003</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>

    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.3 0.7 0.9 1</background>
      <shadows>false</shadows>
      <grid>false</grid>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 8 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.3 0.2 -1</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>
          <surface>
            <friction>
              <ode>
                <mu>1.0</mu>
                <mu2>1.0</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>
          <material><ambient>0.75 0.70 0.60 1</ambient><diffuse>0.75 0.70 0.60 1</diffuse></material>
        </visual>
      </link>
    </model>
"""]
    for name, cx, cy, sx, sy in WALLS:
        parts.append(box_link(name, cx, cy, sx, sy, WALL_H, (0.92, 0.92, 0.90)))
    for name, cx, cy, sx, sy, h, rgb in BOXES:
        parts.append(box_link(name, cx, cy, sx, sy, h, rgb))
    for name, uri, x, y, yaw, _ in FUEL_OBJECTS:
        parts.append(f"""
    <include>
      <name>{name}</name>
      <pose>{x} {y} 0 0 0 {yaw}</pose>
      <uri>{uri}</uri>
    </include>""")
    parts.append("\n  </world>\n</sdf>\n")
    return ''.join(parts)


def build_map():
    w, h = int(MAP_W / RES), int(MAP_H / RES)
    img = np.full((h, w), 205, dtype=np.uint8)            # 205 = unknown

    def to_px(x, y):
        return int(round((x - MAP_X0) / RES)), int(round(h - (y - MAP_Y0) / RES))

    def fill(cx, cy, sx, sy, val):
        c0, r1 = to_px(cx - sx / 2, cy - sy / 2)
        c1, r0 = to_px(cx + sx / 2, cy + sy / 2)
        img[max(r0, 0):min(r1, h), max(c0, 0):min(c1, w)] = val

    fill(0, 0, 2 * ROOM_X, 2 * ROOM_Y, 254)               # free space trong phòng
    for _, cx, cy, sx, sy in WALLS:
        fill(cx, cy, sx, sy, 0)
    for _, cx, cy, sx, sy, *_ in BOXES:
        fill(cx, cy, sx, sy, 0)
    for _, _, x, y, _, r in FUEL_OBJECTS:
        fill(x, y, 2 * r, 2 * r, 0)

    os.makedirs(MAP_DIR, exist_ok=True)
    Image.fromarray(img, mode='L').save(os.path.join(MAP_DIR, 'home_room.pgm'))
    with open(os.path.join(MAP_DIR, 'home_room.yaml'), 'w') as f:
        f.write(f"image: home_room.pgm\nmode: trinary\nresolution: {RES}\n"
                f"origin: [{MAP_X0}, {MAP_Y0}, 0]\nnegate: 0\n"
                "occupied_thresh: 0.65\nfree_thresh: 0.25\n")


if __name__ == '__main__':
    os.makedirs(WORLD_DIR, exist_ok=True)
    with open(os.path.join(WORLD_DIR, 'home_room.sdf'), 'w') as f:
        f.write(build_sdf())
    build_map()
    print('Đã sinh worlds/home_room.sdf và maps/home_room.{pgm,yaml}')
