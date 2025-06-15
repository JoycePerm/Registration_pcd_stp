from OCC.Core.STEPControl import STEPControl_Reader, STEPControl_Writer, STEPControl_AsIs  
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopoDS import TopoDS_Solid, topods_Solid, TopoDS_Compound
from OCC.Core.TopAbs import TopAbs_SOLID
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCC.Core.BRep import BRep_Builder
from OCC.Core.GProp import GProp_GProps
from OCC.Core.BRepGProp import brepgprop_VolumeProperties
from OCC.Core.BRepBndLib import brepbndlib_Add
from OCC.Core.Bnd import Bnd_Box
from OCC.Core.gp import gp_Pnt, gp_Ax1, gp_Dir, gp_Trsf

# 读取STP文件
def read_stp(stp_file_path):
    step_reader = STEPControl_Reader()
    status = step_reader.ReadFile(stp_file_path)
    
    if status != 1:  # 1表示成功读取‌:ml-citation{ref="1,2" data="citationList"}
        raise ValueError(f"文件读取失败，错误码：{status}")
    step_reader.TransferRoots()  # 转换几何数据‌:ml-citation{ref="1,2" data="citationList"}
    return step_reader


# 提取所有实体
def get_solids(step_reader):
    shape = step_reader.OneShape()
    explorer = TopExp_Explorer(shape, TopAbs_SOLID)
    solids = []
    
    while explorer.More():
        current_solid = topods_Solid(explorer.Current())
        if not current_solid.IsNull():
            solids.append(current_solid)
        explorer.Next()
    return solids

def get_volume_props(entity):
    """计算体积和质心"""
    props = GProp_GProps()
    brepgprop_VolumeProperties(entity, props)
    return (
        round(props.Mass(), 4),          # 体积，保留4位小数
        tuple(round(c, 4) for c in props.CentreOfMass().Coord())  # 质心XYZ坐标
    )

def sort_entities(entities):
    """按体积升序排序，体积相同则按质心XYZ字典序排序"""
    # 提取属性并保存原始实体引用
    entities_with_props = []
    for ent in entities:
        vol, centroid = get_volume_props(ent)
        entities_with_props.append( (vol, centroid, ent) )
    
    # 多级排序：体积 -> 质心X -> 质心Y -> 质心Z
    sorted_entities = sorted(
        entities_with_props,
        key=lambda x: (x[0], x[1])
    )
    return [ent for vol, centroid, ent in sorted_entities]

# 保存实体到新STP
def save_solids_to_step(solids, output_path):
    writer = STEPControl_Writer()
    
    # 创建复合体容器
    builder = BRep_Builder()
    compound = TopoDS_Compound()
    builder.MakeCompound(compound)  # 初始化空复合体‌:ml-citation{ref="1,2" data="citationList"}
    
    # 逐个添加实体
    for solid in solids:
        if not solid.IsNull():  # 有效性检查‌:ml-citation{ref="1" data="citationList"}
            builder.Add(compound, solid)  # 将实体加入容器‌:ml-citation{ref="1" data="citationList"}
    
    # 写入文件
    writer.Transfer(compound, STEPControl_AsIs)  # 传输复合体到写入器‌:ml-citation{ref="2" data="citationList"}
    status = writer.Write(output_path)
    
    if status != 1:
        raise RuntimeError(f"文件保存失败，错误码：{status}")


# 应用变换矩阵
def apply_transformation_to_stp(shape, transformation_matrix):
    # 验证矩阵维度
    if transformation_matrix.shape != (4, 4):
        raise ValueError("需输入4x4齐次变换矩阵")
    
    transformation_matrix = transformation_matrix[:3].reshape(-1).tolist()
    # 创建变换矩阵
    trsf = gp_Trsf()
    trsf.SetValues(*transformation_matrix)
    
    # 执行几何变换
    brep_transform = BRepBuilderAPI_Transform(shape, trsf)
    if not brep_transform.IsDone():
        raise RuntimeError("变换操作失败")
    
    return brep_transform.Shape()

def is_flip(entities):
    max_z, max_volume = 0, 0
    max_z_idx, max_volume_idx = None, None
    for idx, entity in enumerate(entities):
        # 计算包围盒
        bbox = Bnd_Box()
        brepbndlib_Add(entity, bbox)

        # 获取包围盒的最小点和最大点
        x_min, y_min, z_min, x_max, y_max, z_max = bbox.Get()
        if z_max > max_z:
            max_z = z_max
            max_z_idx = idx
         # 计算实体的体积
        volume_props = GProp_GProps()
        brepgprop_VolumeProperties(entity, volume_props)
        volume = volume_props.Mass()
        if volume > max_volume:
            max_volume = volume
            max_volume_idx = idx

    if max_z_idx is not None and max_volume_idx is not None and max_z_idx == max_volume_idx:
        print(max_z_idx, max_volume_idx)
        return True
    return False

def flip(entities):
    trans_entities = []
    # 定义镜像变换：沿 Z 轴镜像
    mirror_transform = gp_Trsf()
    mirror_transform.SetMirror(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)))  # 镜像平面为 XY 平面
    for entity in entities:
        # 上下翻转几何体（沿 Z 轴镜像）
        transformed_shape = BRepBuilderAPI_Transform(entity, mirror_transform, True).Shape()
        trans_entities.append(transformed_shape)
    return trans_entities


def trans_stp(stp, trans_matrix, save_file_path = None):
    if not isinstance(stp, list):
        step_reader = read_stp(stp)
        solid_list = get_solids(step_reader)
        #sorted_solid_list = sort_entities(solid_list)
    else:
        solid_list = stp

    for n in range(len(solid_list)):
        solid_list[n] = apply_transformation_to_stp(solid_list[n], trans_matrix)
    # if is_flip(solid_list):
    #     flip(solid_list)    
    if save_file_path is not None: 
        save_solids_to_step(solid_list, save_file_path)

    return solid_list
