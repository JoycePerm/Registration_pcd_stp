from OCC.Core.STEPControl import STEPControl_Reader
from OCC.Core.TopAbs import (
    TopAbs_COMPOUND,
    TopAbs_COMPSOLID,
    TopAbs_SOLID,
    TopAbs_SHELL,
    TopAbs_FACE,
    TopAbs_WIRE,
    TopAbs_EDGE,
    TopAbs_VERTEX,
)
from OCC.Core.TopExp import TopExp_Explorer

from OCC.Core.STEPControl import STEPControl_Reader, STEPControl_AsIs
from OCC.Core.StlAPI import StlAPI_Writer
from OCC.Core.BRepMesh import BRepMesh_IncrementalMesh
from OCC.Core.STEPControl import STEPControl_Reader, STEPControl_Writer
from OCC.Core.STEPControl import STEPControl_Reader
import math
import numpy as np

from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Fuse
import open3d as o3d
import os
from contextlib import contextmanager

@contextmanager
def modified_file_logger(log_path=r"/home/chenjiayi/peizhun/stp_to_stl_failed.txt"):
    """上下文管理器：自动管理modified.txt文件写入"""
    try:
        with open(log_path, 'a', encoding='utf-8') as log_file:
            yield log_file  # 将文件对象传递给外部代码
    except IOError as e:
        print(f"无法写入日志文件 {log_path}: {str(e)}")
        raise

def get_solids_in_compound(compound_shape):
    """
    获取复合形状中包含的实体
    """
    solids = []
    topology_explorer = TopExp_Explorer(compound_shape, TopAbs_SOLID)
    while topology_explorer.More():
        solid = topology_explorer.Current()
        solids.append(solid)
        topology_explorer.Next()
    return solids

def merge_solids(solids):
    if len(solids) == 0:
        return None
    result_solid = solids[0]
    for solid in solids[1:]:
        fuse = BRepAlgoAPI_Fuse(result_solid, solid)
        fuse.Build()
        # result_solid = topods.Solid(fuse.Shape())
        result_solid = fuse.Shape()
    return result_solid

def get_solids(step_reader: STEPControl_Reader):
    """
    读取STEP文件并遍历其内部拓扑结构，重点获取复合形状中的实体以及实体包含面的信息
    """
    solid_list = []
    for n in range(1, step_reader.NbShapes() + 1):
        shape = step_reader.Shape(n)
        # 创建一个拓扑结构探索器，用于遍历形状的拓扑结构
        topology_explorer = TopExp_Explorer(shape, TopAbs_COMPOUND)

        s_list = []
        while topology_explorer.More():
            sub_shape = topology_explorer.Current()

            # 获取子形状的类型（比如是实体、面、边等）
            shape_type = sub_shape.ShapeType()
            if shape_type == TopAbs_COMPOUND:
                # print("发现复合形状")
                solids_in_compound = get_solids_in_compound(sub_shape)
                s_list.extend(solids_in_compound)

            else:
                raise Exception("shape type", shape_type)

            topology_explorer.Next()
        if len(s_list) == 0:
            s_list = get_solids_in_compound(shape)
        solid_list.extend(s_list)
    return solid_list

def read_stp(file_path):
    # 创建STEP读取器对象
    step_reader = STEPControl_Reader()
    # 读取STEP文件，返回状态码，若成功读取，状态码为IFSelect_RetDone
    status = step_reader.ReadFile(file_path)
    if status == 1:
        # 传输数据，将文件内容加载到内存中准备后续处理
        step_reader.TransferRoots()
    else:
        print("无法正确读取STEP文件:", file_path)
        return None

    return step_reader
def stp_to_stl(stp_file_path, stl_file_path, linear_deflection=0.0001):
    step_reader = read_stp(stp_file_path)
    if step_reader is None:
        print("step_reader")
        return None
    solid_list = get_solids(step_reader)
    
    
    try:
        # print(len(solid_list), shape)
        # 对形状进行网格划分
        shape = merge_solids(solid_list)
        mesh = BRepMesh_IncrementalMesh(shape, linear_deflection)
        mesh.Perform()
        if not mesh.IsDone():
            print("Mesh generation failed")
            return
        # 写入 STL 文件
        stl_writer = StlAPI_Writer()
        stl_writer.Write(shape, stl_file_path)
    except Exception as e:
        print("stp_to_stl failed:", stp_file_path)
        with modified_file_logger() as log_file:
            log_file.write(f"stp_to_stl failed:{stp_file_path}""\n")
    return 1


def main():
    path = r"\\10.1.1.102\26.CypWeld项目组\13.AI点云资料\规范化数据\实验室数据\pre_downsample_20250309"

    '''
    stp_files = [file for file in os.listdir(stp_path) if file.lower().endswith(".stp")]
    stl_files = [file for file in os.listdir(stl_path) if file.lower().endswith(".stl")]
    for stp_file in stp_files:
        stl_file_name = stp_file.split('.')[0] + '.stl'
        if stl_file_name not in stl_files:
            stp_file_path = os.path.join(stp_path, stp_file)
            stl_file_path = os.path.join(stl_path, stl_file_name)
            stp_to_stl(stp_file_path, stl_file_path)
    '''
    for i in range(400, 410):
        stp_file_path = path + '\\' +  str(i) + '\\3d_model.step'
        stl_file_path = path + '\\' +  str(i) + '\\3d_model.stl'
        stp_to_stl(stp_file_path, stl_file_path)



if __name__ == "__main__":
    main()