#!/usr/bin/env python3
import rospy, random
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import *
from urdf_parser_py import urdf 
from std_srvs.srv import Empty
from xml.dom import minidom
from rospkg.rospack import RosPack
import copy
from datetime import datetime
class SpawnModels():
    def __init__(self):
        rospy.init_node("spawn_model")
        rospy.wait_for_service("gazebo/delete_model")
        rospy.wait_for_service("gazebo/spawn_urdf_model")
        rospy.wait_for_service("gazebo/unpause_physics")
        self.delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
        self.spawn_urdf_model = rospy.ServiceProxy("gazebo/spawn_urdf_model", SpawnModel)
        self.unpause_physics = rospy.ServiceProxy("gazebo/unpause_physics", Empty)
        self.pause_physics = rospy.ServiceProxy("gazebo/pause_physics",Empty)
        self.pause_physics()
        self.readXml()
        self.spawnModel()
        print("about to unpause")
        
        self.unpause_physics()
        print("unpause complete")
        rospy.set_param("spawn_complete",1)
        # rospy.on_shutdown(self.myhook)
        
    def myhook(self):
        print("shutdown time!")
        
    def readXml(self):
        # parse an xml file by name
        specific_file_path = "/config/item_spawn.xml"
        xml_path = RosPack().get_path('ur5_data_collect_fw')+specific_file_path
        file = minidom.parse(xml_path)
        prefull_file = list()
        self.full_file = list()
        #use getElementsByTagName() to get tag
        models = file.getElementsByTagName('model')
        unknown_object_name = 0
        name_dict = dict()
        for model in models:
            temp = FilledObject("","","","","","","","","","","","")
            # temp = FilledObject("a","a","a","a","a","a","a","a","a","a","a","a")
            if model.hasAttribute('id'):
                name = model.getAttribute('id')
            else:
                unknown_object_name += 1
                name = "object_"+str(unknown_object_name)
            if name in name_dict:
                name_dict[name] += 1
                name = name + "_" + chr(64+name_dict[name])
                name_dict[name] = 0
            else:
                name_dict[name] = 0
            temp.id = name
            if model.hasAttribute('type'):
                filetype = model.getAttribute('type')
                if filetype.lower() == "box":
                    if model.hasAttribute('box_size'):
                        box_size = model.getAttribute('box_size')
                        if len(box_size.split())!=3 or len([True for i in box_size.split() if i.count(".")<=1])<3 or \
                        len(box_size)!=len([True for i in box_size if i in ". 0123456789"]): 
                            temp.box_size = 'error, please specify 3 float, "0.5 0.5 0.5"'
                            temp.error = True
                        else:
                            temp.box_size = box_size
                elif filetype.lower() == "cylinder":
                    if model.hasAttribute('radius'):
                        radius = model.getAttribute('radius')
                        if len(radius)!=len([True for i in radius.strip() if i in ".0123456789"]) or radius.count(".")>1: 
                            temp.radius = 'error, please specify 1 float'
                            temp.error = True
                        else:
                            temp.radius = radius
                    if model.hasAttribute('length'):
                        length = model.getAttribute('length')
                        if len(length)!=len([True for i in length.strip() if i in ".0123456789"]) or length.count(".")>1: 
                            temp.length = 'error, please specify 1 float'
                            temp.error = True
                        else:
                            temp.length = length
            elif model.hasAttribute('filename'):
                filename = model.getAttribute('filename')
                temp.filename = filename
                filetype = filename[filename.rfind(".")+1:]
            if filetype.lower() in ["urdf","stl","dae","box","cylinder"]:
                temp.type = filetype
            else:
                temp.type = "error, invalid type"
                temp.error = True
            if model.hasAttribute('mass'):
                mass = model.getAttribute('mass')
                if len(mass)!=len([True for i in mass.strip() if i in ".0123456789"]) or mass.count(".")>1: 
                    temp.mass = 'error, please specify 1 float'
                    temp.error = True
                else:
                    temp.mass = mass
            if model.hasAttribute('mu1'):
                mu1 = model.getAttribute('mu1')
                if len(mu1)!=len([True for i in mu1.strip() if i in ".0123456789"]) or mu1.count(".")>1: 
                    temp.mu1 = 'error, please specify 1 float'
                    temp.error = True
                else:
                    temp.mu1 = mu1
            if model.hasAttribute('mu2'):
                mu2 = model.getAttribute('mu2')
                if len(mu2)!=len([True for i in mu2.strip() if i in ".0123456789"]) or mu2.count(".")>1: 
                    temp.mu2 = 'error, please specify 1 float'
                    temp.error = True
                else:
                    temp.mu2 = mu2
            if model.hasAttribute('color'):
                temp.color = model.getAttribute('color')
            if model.hasAttribute('xyz'):
                xyz = model.getAttribute('xyz')
                if len(xyz.split())!=3 or len([True for i in xyz.split() if i.count(".")<=1])<3 or \
                len(xyz)!=len([True for i in xyz if i in ". 0123456789"]): 
                    temp.xyz = 'error, please specify 3 float, "0.5 0.5 0.5"'
                    temp.error = True
                else:
                    temp.xyz = xyz
            if model.hasAttribute('amount'):
                amount = model.getAttribute("amount")
                if amount.isdigit() and not "_" in amount:
                    if int(amount) >= 1:
                        temp.amount = amount
                    elif int(amount) == 0:
                        temp.amount = "0"
                        temp.error = True  
                    else: 
                        temp.amount = "error, invalid amount"
                        temp.error = True    
                else:
                    temp.amount = "error, invalid amount"
                    temp.error = True                    
            else:
                temp.amount = "1"
            prefull_file.append(temp)
            ##createXml(prefull_file)
        
        randXyz = RandomXyz()
        for model in prefull_file:
            models = copy.deepcopy(model)
            amount = models.amount
            name = models.id
            if amount.isdigit():
                if int(amount) > 1:
                    plus_number = 0
                    for i in range(int(amount)):
                        models = copy.deepcopy(model)
                        models.id = name + "_" + str(i+plus_number)
                        while models.id in name_dict:
                            plus_number += 1
                            models.id = name + "_" + str(i+plus_number)    
                        models.amount = 1
                        if models.xyz == "":
                            models.xyz = randXyz.xyz()
                        self.full_file.append(models)
                else:
                    if models.xyz == "":
                        models.xyz = randXyz.xyz()
                    self.full_file.append(models)
            else:
                if models.xyz == "":
                    models.xyz = randXyz.xyz()
                self.full_file.append(models)
        
        self.exportXml(self.full_file,True)  
    
    def exportXml(self, object_list, sorting=False):
        if sorting:
            name_list = []
            for num, val in enumerate(object_list):
                # print(num, val.id)
                Id = [e[::-1] for e in val.id[::-1].partition("_")][::-2]
                if Id[0] == "":
                    Id = Id[::-1]
                elif not Id[1].isdigit():
                    Id = ["_".join(Id),""]
                if Id[1] == "":
                    Id[1] = "-1"
                name_list.append([num, Id])
            object_order = [obj[0] for obj in sorted(name_list, key=lambda x:(x[1][0], int(x[1][1])))]
            # name_order = [obj[1] for obj in sorted(name_list, key=lambda x:(x[1][0], int(x[1][1])))]
        else:
            object_order = range(len(object_list))
        
        doc = minidom.Document()
        doc.appendChild(doc.createComment("Full Filled XML Document"))

        root = doc.createElement('root')
        doc.appendChild(root)
        
        models = doc.createElement('models')
        root.appendChild(models)
        attributeList = ["id","type","color","xyz","filename","amount","mass","box_size","radius","length","mu1","mu2"]
        for order in object_order:
            obj = object_list[order]
            if not obj.error:
                model = doc.createElement('model')
                models.appendChild(model)
                for attr in attributeList:
                    if eval("obj."+attr) != "":
                        model.setAttribute(attr, str(eval("obj."+attr)))
        #open & write text file
        path = RosPack().get_path('ur5_data_collect_fw') + '/rec/' + str(datetime.now()) + '.xml'
        with open(path, "w") as text_file:
            text_file.write(doc.toprettyxml(indent = '   '))        
        
    def createObjectXml(self,geometry="box", color="Green", mass=1.0,
                    box_size="0.05 0.05 0.05",
                    radius="0.025", length="0.05",
                    mu1=1.0, mu2=1.0):
        
        item = urdf.Robot(name=geometry+"_"+color)
        if color == "":
            color = "Green"
        else:
            color = color[0].upper() + color[1:].lower()
        if mass == "":
            mass = 1.0
        else:
            mass = float(mass)
        if mu1 == "":
            mu1 = 1.0
        if mu2 == "":
            mu2 = 1.0
        if geometry == "box":
            if box_size == "":
                box_size = "0.05 0.05 0.05"
            item = urdf.Robot(name=geometry+"_"+color)
            length = [float(num) for num in box_size.split()]
            shape = urdf.Box(length)
            inertia = urdf.Inertia(1/12*mass*(length[1]**2+length[2]**2),0,0,
                                1/12*mass*(length[0]**2+length[2]**2),0,
                                1/12*mass*(length[0]**2+length[1]**2))
            
        elif geometry == "cylinder":
            if radius == "":
                radius = 0.025
            else:      
                radius = float(radius)
            if length == "":
                length = 0.05
            else:
                length = float(length)
            item = urdf.Robot(name=geometry+"_"+color)
            shape = urdf.Cylinder(float(radius),float(length))
            inertia = urdf.Inertia(1/12*mass*(3*radius**2+length**2),0,0,
                                1/12*mass*(3*radius**2+length**2),0,
                                1/2*mass*radius**2)
        ##elif sphere, mesh
        else:
            print("Invalid Geometry")
        
        item.add_link(urdf.Link("link",urdf.Visual(shape), 
                                urdf.Inertial(mass,inertia),
                                urdf.Collision(shape)))
        
        item_xml = item.to_xml_string()
        item_xml = item_xml[:item_xml.find("</robot>")] + \
        """  <gazebo reference="link">
            <material>Gazebo/{color}</material>
            <mu1>{mu1}</mu1>
            <mu2>{mu2}</mu2>
        </gazebo>
        </robot>"""
        
        return item_xml.format(color=color, mu1=float(mu1), mu2=float(mu2))

    def spawnModel(self):
        for model in self.full_file:
            if not model.error:
                self.delete_model(model.id)
        time_delay(2)
        for model in self.full_file:
            if not model.error:        
                if model.type.lower() in ["box","cylinder"]:
                    product_xml = self.createObjectXml(geometry=model.type.lower(), 
                                                       color=model.color, mass=model.mass,
                                                       box_size=model.box_size, radius=model.radius,
                                                       length=model.length, mu1=model.mu1, mu2=model.mu2)     
                elif model.type.lower() == "urdf":
                    path = RosPack().get_path('ur5_data_collect_fw')+model.filename
                    product_xml = minidom.parse(path).toxml()
                # elif model.type.lower() in ["stl","dae"]:
                item_pose = Pose()
                if model.xyz != "":
                    pose = [float(num) for num in model.xyz.split()]
                    item_pose.position.x = pose[0]
                    item_pose.position.y = pose[1]
                    item_pose.position.z = pose[2]
                else:
                    item_pose.position.x = 0.4
                    item_pose.position.y = 0.1
                    item_pose.position.z = 2
                self.spawn_urdf_model(model.id, product_xml, "", item_pose, "ground_plane::link")

class FilledObject():
    
    def __init__(self, id, type, color, xyz, filename,amount,mass,box_size,radius,length,mu1,mu2):
        self.id = id
        self.type = type
        self.color = color
        self.xyz = xyz
        self.filename = filename
        self.amount = amount
        self.mass = mass
        self.box_size = box_size
        self.radius = radius
        self.length = length
        self.mu1 = mu1
        self.mu2 = mu2
        self.error = False
        
def time_delay(sec):
    now = datetime.now()
    delta_time = datetime.now() - now
    while delta_time.seconds < sec:
        delta_time = datetime.now() - now
        
class RandomXyz():
    
    def __init__(self):
        desk_file = minidom.parse(RosPack().get_path('ur5_data_collect_fw')+"/urdf/desk.xacro")
        desk_property = desk_file.getElementsByTagName('xacro:property')
        desk_dict = dict()
        for prop in desk_property:
            desk_dict[prop.getAttribute("name")] = prop.getAttribute("value")
        self.width = float(desk_dict['plate_width'])
        self.length = float(desk_dict['plate_length'])
        self.height = float(desk_dict['desk_height'])
        
    def xyz(self):
        x = 0.45 * self.width * (-1+2*random.random())
        y = 0.45 * self.length * (-1+2*random.random())
        z = 1.5 * self.height
        return (" ").join([str(x),str(y),str(z)])

if __name__ == "__main__":
    try:
        SpawnModels()
        
    except:
        pass