# TurtleBot3 Coverage Simulation using Behavior Tree (ROS Noetic)

## Deskripsi Proyek
Repository ini berisi **dokumentasi dan implementasi simulasi TurtleBot3 (Burger)** menggunakan **ROS Noetic** dan **Gazebo**, dengan pendekatan **Behavior Tree (BT)** sebagai sistem pengambilan keputusan (*decision making*).

Robot disimulasikan sebagai **robot pembersih lantai (floor cleaning robot)** yang memiliki kemampuan:
- Bergerak maju secara otonom
- Menghindari rintangan
- Menghentikan pergerakan ketika misi selesai
- Melakukan *coverage cleaning* hingga persentase tertentu terpenuhi
- Memvisualisasikan area yang telah dibersihkan menggunakan **RViz**

---

## Teknologi yang Digunakan
- ROS Noetic
- Gazebo Simulator
- TurtleBot3 (Burger)
- Python (ROS Node)
- Behavior Tree
- RViz

---

## Struktur Sistem
Sistem dijalankan menggunakan **3 terminal utama**:

1. **Terminal 1**  
   Menjalankan simulasi TurtleBot3 di Gazebo

2. **Terminal 2**  
   Menjalankan node Behavior Tree (`bt_node.py`) sebagai pengendali utama robot

3. **Terminal 3**  
   Menjalankan node pemetaan coverage (`coverage_tracker.py`) dan visualisasi di RViz

---

## Alur Kerja Sistem
1. Simulasi Gazebo dijalankan dengan model TurtleBot3 Burger.
2. Node Behavior Tree membaca kondisi lingkungan (rintangan dan status coverage).
3. Robot bergerak maju selama tidak ada rintangan.
4. Ketika rintangan terdeteksi:
   - Robot berhenti
   - Robot berotasi untuk menghindari rintangan
5. Robot terus bergerak dan membersihkan area.
6. Posisi robot dipetakan ke dalam grid coverage.
7. Coverage dihitung berdasarkan jumlah sel grid yang telah dilewati robot.
8. Ketika coverage target terpenuhi, robot berhenti otomatis.

---

## Behavior Tree Design
Behavior Tree menggunakan **Priority Selector** dengan urutan prioritas sebagai berikut:

1. **Coverage Selesai**  
   Jika coverage sudah terpenuhi → robot berhenti

2. **Rintangan Terdeteksi**  
   Robot berhenti lalu berotasi untuk menghindari rintangan

3. **Kondisi Normal**  
   Robot bergerak maju

### Cuplikan Logika Behavior Tree
```python
if self.coverage_done.check():
    self.stop.run()

elif self.obstacle.is_danger():
    self.stop.run()
    self.rotate.run()

else:
    self.move.run()
```python
###

## Cara Menjalankan Program

### 1. Persiapan Workspace
Pastikan workspace ROS sudah berada di direktori yang benar dan semua dependency telah terpasang.

Masuk ke folder workspace:
```bash
cd ~/catkin_ws
```bash
catkin build
```bash
source devel/setup.bash
```bash

### 2. Terminal 1 – Menjalankan Simulasi Gazebo
Buka terminal baru dan jalankan simulasi TurtleBot3 di Gazebo:
```bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_world.launch

Terminal ini akan menampilkan:
Dunia simulasi Gazebo
Robot TurtleBot3 Burger