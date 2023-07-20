from stlib3.physics.deformable import ElasticMaterialObject
from stlib3.physics.collision import CollisionMesh
from stlib3.scene import MainHeader, ContactHeader

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
                        translation=[0.05,0.03,0.0],
                        rotation=[0.0,0.0, 0.0],
                        showObjectScale=10)

        floor.addObject('UniformMass', name='mass', vertexMass=[totalMass, volume, inertiaMatrix[:]])
     
        #Floor Collision

        fCollision = floor.addChild('Collision')
        fCollision.addObject('MeshObjLoader', name='mloader', filename=floorMeshPath, triangulate="true", scale=10)
        fCollision.addObject('MeshTopology', src='@mloader')
        fCollision.addObject('MechanicalObject')

        fCollision.addObject('TriangleCollisionModel')
        fCollision.addObject('LineCollisionModel')
        fCollision.addObject('PointCollisionModel')        

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
                     youngModulus=1.0e+03,
                     position=[0.0,0.0,0.0],
                     rotation=[-90.0,0.0,0.0],
                     color=[0.16,0.82,0.95, 1]):
        
        defObject = rootNode.addChild(f'Deformable{form}')

        eobject = ElasticMaterialObject(defObject,
                                        volumeMeshFileName=f"mesh/{form}.vtk",
                                        poissonRatio=poissonRatio,
                                        youngModulus=youngModulus,
                                        totalMass=totalMass,
                                        surfaceColor=color,
                                        surfaceMeshFileName=f"mesh/{form}.obj",
                                        rotation=rotation,
                                        translation=position)
        
        CollisionMesh(eobject, name="CollisionMesh",
                      surfaceMeshFileName=f"mesh/{form}.obj",
                      rotation=rotation, translation=position)
        
        defObject.addChild(eobject)
        
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

        barrel = rootNode.addChild(f'Rigid{form}')
        barrel.addObject('MechanicalObject', name='mstate', translation=position, rotation=rigidRotation, template='Rigid3', showObjectScale=scale)
        barrel.addObject('UniformMass', name='mass', vertexMass=[totalMass, volume, inertiaMatrix[:]])
        barrel.addObject('UncoupledConstraintCorrection')
        barrel.addObject('EulerExplicitSolver', name='Solver')
        barrel.addObject('CGLinearSolver', name='LinearSolver')
        
        collision = barrel.addChild('Collision')
        collision.addObject('MeshObjLoader', name='loader', filename=meshPath, triangulate="true", scale=scale) # Loads the mesh
        collision.addObject('MeshTopology', src='@loader') #Defines the topology
        collision.addObject('MechanicalObject')
        collision.addObject('TriangleCollisionModel')
        collision.addObject('LineCollisionModel')
        collision.addObject('PointCollisionModel')
        collision.addObject('RigidMapping')

        ## visualization

        visualization = barrel.addChild('VisualModel')
        visualization.loader = visualization.addObject('MeshObjLoader', name='loader', filename=meshPath, scale=1)
        visualization.addObject('OglModel', name='model', src='@loader', scale3d=[scale]*3, color=color, updateNormals=False)
        visualization.addObject('RigidMapping')
 
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
               plugins=pluginList
               )
    
    ContactHeader(rootNode, 
                  alarmDistance=.5, 
                  contactDistance=.2, 
                  frictionCoef=0.08)   

    rootNode.findData('dt').value=0.01
    rootNode.findData('gravity').value= [0, -9810, 0]
 
    floor(rootNode)

    list_of_objects = ["Cube",
                       "Cone",
                       "Sphere",
                       "Cylinder"]

    deformableObject(rootNode,
                     form = list_of_objects[1],
                     position = [-5.0,1.0,0.0]
                     )

    rigidObject(rootNode,
                form = list_of_objects[2],
                position = [5.0,1.0,0.0]
                )
    
#     rootNode.removeObject(rootNode.GenericConstraintSolver)

    rootNode.addObject('LightManager')
    rootNode.addObject('PositionalLight', name="light1", color="0.8 0.8 0.8", position="0 60 -50")
    rootNode.addObject('PositionalLight', name="light2", color="0.8 0.8 0.8", position="0 -60 50")

    return rootNode
