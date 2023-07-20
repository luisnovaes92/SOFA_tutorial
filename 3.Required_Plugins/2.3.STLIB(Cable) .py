from stlib3.physics.deformable import ElasticMaterialObject
from stlib3.physics.collision import CollisionMesh
from stlib3.scene import MainHeader, ContactHeader
from stlib3.physics.constraints import FixedBox
from softrobots.actuators import PullingCable

def floor(rootNode):

        totalMass = 1.0
        volume = 1.0
        inertiaMatrix=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        floorMeshPath = 'mesh/floor.obj'
        floorColor = [0.5,0.6,0.8]

        floor = rootNode.addChild('Floor')
        floor.addObject('MechanicalObject',
                        name='mstate',
                        template='Rigid3',
                        translation=[0.05,-4.03,0.0],
                        rotation=[0.0,0.0, -30.0],
                        showObjectScale=10)

        floor.addObject('UniformMass', name='mass', vertexMass=[totalMass, volume, inertiaMatrix[:]])
     
        #Floor Collision

        fCollision = floor.addChild('Collision')
        fCollision.addObject('MeshObjLoader', name='mloader', filename=floorMeshPath, triangulate="true", scale=10)
        fCollision.addObject('MeshTopology', src='@mloader')
        fCollision.addObject('MechanicalObject')

        fCollision.addObject('TriangleCollisionModel', moving=False, simulated=True)
        fCollision.addObject('LineCollisionModel', moving=False, simulated=True)
        fCollision.addObject('PointCollisionModel', moving=False, simulated=True)        

        fCollision.addObject('RigidMapping')

        #Floor VisualModel

        fVisual = floor.addChild('VisualModel')
        fVisual.loader = fVisual.addObject('MeshObjLoader', name='mloader', filename=floorMeshPath)
        fVisual.addObject('OglModel', name='model', src='@mloader', scale3d=[10]*3, color=floorColor, updateNormals=False)
        fVisual.addObject('RigidMapping')

def deformableObject(rootNode,
                     form='', 
                     totalMass=.01, 
                     poissonRatio=0.45,
                     youngModulus=1.0e+04,
                     position=[0.0,0.0,0.0],
                     deformableRotation=[-90.0,0.0,0.0],
                     color=[0.16,0.82,0.95, 1]):
        
        defObject = rootNode.addChild(f'Deformable{form}')

        eobject = ElasticMaterialObject(defObject,
                                        volumeMeshFileName=f"mesh/{form}.vtk",
                                        poissonRatio=poissonRatio,
                                        youngModulus=youngModulus,
                                        totalMass=totalMass,
                                        surfaceColor=color,
                                        surfaceMeshFileName=f"mesh/{form}.obj",
                                        rotation=deformableRotation,
                                        translation=position)
        
        CollisionMesh(eobject, name="CollisionMesh",
                      surfaceMeshFileName=f"mesh/{form}.obj",
                      rotation=deformableRotation, translation=position)
        
        defObject.addChild(eobject)

        return eobject
        
def rigidObject(rootNode,
                form='', 
                totalMass=.01, 
                volume=.01,
                inertiaMatrix=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                position = [0.0,0.0,0.0],
                rigidRotation = [-90.0,0.0,0.0],
                color=[0.02,0.44,0.95],
                scale=1):
       
        meshPath = f'mesh/{form}.obj'             

        rigidObject = rootNode.addChild(f'Rigid{form}')
        rigidObject.addObject('MechanicalObject', name='mstate', translation=position, rotation=rigidRotation, template='Rigid3', showObjectScale=scale)
        rigidObject.addObject('UniformMass', name='mass', vertexMass=[totalMass, volume, inertiaMatrix[:]])
        rigidObject.addObject('UncoupledConstraintCorrection')
        rigidObject.addObject('EulerExplicitSolver', name='Solver')
        rigidObject.addObject('CGLinearSolver', name='LinearSolver')
        # rigidObject.addObject('FixedConstraint')
        
        collision = rigidObject.addChild('Collision')
        collision.addObject('MeshObjLoader', name='loader', filename=meshPath, triangulate="true", scale=scale) # Loads the mesh
        collision.addObject('MeshTopology', src='@loader') #Defines the topology
        collision.addObject('MechanicalObject')
        collision.addObject('TriangleCollisionModel')
        collision.addObject('LineCollisionModel')
        collision.addObject('PointCollisionModel')
        collision.addObject('RigidMapping')

        ## visualization

        visualization = rigidObject.addChild('VisualModel')
        visualization.loader = visualization.addObject('MeshObjLoader', name='loader', filename=meshPath, scale=1)
        visualization.addObject('OglModel', name='model', src='@loader', scale3d=[scale]*3, color=color, updateNormals=False)
        visualization.addObject('RigidMapping')

        return rigidObject
 
def createScene(rootNode):

        pluginList = ['Sofa.Component.Constraint.Lagrangian.Correction',
                      'Sofa.Component.Mapping.NonLinear',
                      'Sofa.Component.ODESolver.Forward',
                      "Sofa.Component.AnimationLoop",
                      "Sofa.Component.Collision.Detection.Algorithm",
                      "Sofa.Component.Collision.Detection.Intersection",
                      "Sofa.Component.Collision.Geometry",
                      "Sofa.Component.Collision.Response.Contact",
                      "Sofa.Component.Constraint.Lagrangian.Solver",
                      "Sofa.Component.IO.Mesh",
                      "Sofa.Component.LinearSolver.Iterative",
                      "Sofa.Component.Mass",
                      "Sofa.Component.ODESolver.Backward",
                      "Sofa.Component.StateContainer",
                      "Sofa.Component.Topology.Container.Constant",
                      "Sofa.Component.Visual",
                      "Sofa.GL.Component.Rendering3D"]

        MainHeader(rootNode,
                   gravity=[0.0, -981.0, 0.0], 
                   plugins=pluginList)
        
        ContactHeader(rootNode, 
                      alarmDistance=.5, 
                      contactDistance=.2, 
                      frictionCoef=.08)   

        rootNode.findData('dt').value=0.001
        rootNode.findData('gravity').value= [0, -9810, 0]
        
        floor(rootNode)

        list_of_objects = ["Cube",
                           "Cone",
                           "Sphere",
                           "Cylinder"]

        

        defObject = deformableObject(rootNode,
                                           form = list_of_objects[2],
                                           position = [-5.0,1.0,0.0]
                                           )
        
        FixedBox(defObject, atPositions=[-5.5,3,1,-6,1,-1], doVisualization=True)

        rigObject = rigidObject(rootNode,
                                  form = list_of_objects[0],
                                  position = [5.0,1.0,0.0]
                                  )
          
        cable = PullingCable(defObject,
                             "Pullingcable",
                             translation= [0.036*cos(radians(0)),0.036*sin(radians(0)),0.0],
                             rotation = [0.0,0.0,0.0],
                             cableGeometry=loadPointListFromFile("mesh/Cable.json"),
                             valueType=valueType,
                             pullPointLocation=[0.036*cos(radians(0)),0.036*sin(radians(0)),0.0])


        rootNode.addObject('LightManager')
        rootNode.addObject('PositionalLight', name="light1", color="0.8 0.8 0.8", position="0 60 -50")
        rootNode.addObject('PositionalLight', name="light2", color="0.8 0.8 0.8", position="0 -60 50")

        return rootNode
