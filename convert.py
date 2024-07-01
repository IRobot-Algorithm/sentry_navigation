import open3d as o3d
import os
root_path="/home/niuoruo/Documents"
model_name="output3"

input_path = os.path.join(root_path,model_name+".pcd")
output_path = os.path.join(root_path,model_name+".ply")

# 读取PCD文件
pcd = o3d.io.read_point_cloud(input_path)

# 显示点云以确认读取成功（可选）
# o3d.visualization.draw_geometries([pcd], window_name="PCD Point Cloud", width=800, height=600)

# 保存为PLY文件
o3d.io.write_point_cloud(output_path, pcd)

# print("PCD文件已成功转换为PLY文件。")
