import sys
import Sofa

def createScene(rootNode):
    plugins = [
        # 기본 시뮬레이션 코어 및 빔 모델 관련 플러그인
        "BeamAdapter",
        "Sofa.Component.Constraint.Projective",
        "Sofa.Component.LinearSolver.Direct",
        "Sofa.Component.ODESolver.Backward",
        "Sofa.Component.StateContainer",
        "Sofa.Component.Topology.Container.Constant",
        "Sofa.Component.Topology.Container.Grid",
        "Sofa.Component.Visual",
        "Sofa.Component.SolidMechanics.Spring",
        "Sofa.Component.Topology.Container.Dynamic",
        # 충돌 처리 및 제약 조건 관련 플러그인
        "Sofa.Component.AnimationLoop",
        "Sofa.Component.Collision.Detection.Algorithm",
        "Sofa.Component.Collision.Detection.Intersection",
        "Sofa.Component.Collision.Geometry",
        "Sofa.Component.Collision.Response.Contact",
        "Sofa.Component.Constraint.Lagrangian.Correction",
        "Sofa.Component.Constraint.Lagrangian.Solver",
        "Sofa.Component.IO.Mesh",
        # 시각화 플러그인
        "Sofa.GL.Component.Rendering3D"
    ]
    rootNode.addObject('RequiredPlugin', name="RequiredPlugins", pluginName=" ".join(plugins))
    rootNode.addObject('VisualStyle', displayFlags='showVisualModels hideBehaviorModels showCollisionModels hideMappings showInteractionForceFields')

    # 물리 계산 담당 (충돌 해결을 위해 일단 움직이고 고치는 방식을 사용)
    rootNode.addObject('FreeMotionAnimationLoop')
    # 그래픽 처리 담당 (계산된 결과를 매 프레임마다 화면에 그려줌)
    rootNode.addObject('DefaultVisualManagerLoop')

    # 충돌이나 마찰 같은 제약 조건을 수학적(Linear Complementarity Problem, LCP)으로 풀어냄, nu=마찰 계수, tol=오차 허용 범위, maxIt=최대 반복 횟수
    rootNode.addObject('LCPConstraintSolver', mu='0.1', tolerance='1e-10', maxIt='1000', build_lcp='false')
    # 충돌 처리 관리자, 아래 나오는 것들을 순서대로 실행 시켜줌
    rootNode.addObject('CollisionPipeline', draw='0', depth='6', verbose='1')
    # 1차 검사, 주변에 있는 물체를 대충 확인해줌, N2 모든 물체 쌍을 전부 비교해주는데 정확하고 단순하지만 물체가 많아지면 느려짐
    rootNode.addObject('BruteForceBroadPhase', name='N2')
    # 2차 검사, 부딪혔는지 확인, 1차 검사를 통과한 대상들을 실제 메쉬 단위의 교차 여부 검사 (Bounding Volume Hierarchy, BVH)라는 알고리즘 사용
    rootNode.addObject('BVHNarrowPhase')
    # 근접 감지 및 접촉 생성, contactDistance='0.1': 거리가 0.1 이하가 되면 닿은 것으로 판단, alarmDistance='2': 물체 사이의 거리가 2 이하가 되면 충돌 주의 발령 및 감시 시작
    rootNode.addObject('LocalMinDistance', contactDistance='0.1', alarmDistance='2', name='localmindistance', angleCone='0.2')
    # 충돌이 확인되었을 때 어떻게 처리할지 규칙 생성, response='FrictionContactConstraint': 닿은 부분에 마찰이 있는 접촉 제약을 만들어라, 이 컴포넌트가 제약조건을 만들어서 LCP에 넘겨주는거임
    rootNode.addObject('CollisionResponse', name='Response', response='FrictionContactConstraint')

    # 튜브 형태 설정, 어디서 얼만큼 휘는지
    topoLines = rootNode.addChild('EdgeTopology')

    # 직선 구간 설정
    topoLines.addObject('RodStraightSection', name='StraightSection', 
                                 length=180.0, radius=0.065, 
                                 nbBeams=50, nbEdgesCollis=50, nbEdgesVisu=200, 
                                 youngModulus=20000, massDensity=0.00000155, poissonRatio=0.3)
    
    # 곡선 구간 설정
    topoLines.addObject('RodSpireSection', name='SpireSection', 
                                 length=20.0, radius=0.9, 
                                 nbBeams=10, nbEdgesCollis=10, nbEdgesVisu=200,
                                 spireDiameter=25, spireHeight=0,
                                 youngModulus=20000, massDensity=0.00000155, poissonRatio=0.3)
    
    # 직선, 곡선 합치기
    topoLines.addObject('WireRestShape', name='BeamRestShape', template="Rigid3d",
                                 wireMaterials="@StraightSection @SpireSection")
    
    # 데이터 만들기, 위에서 정의한 직선, 곡선을 컴퓨터가 이해할 수 있도록 데이터 구조로 변환
    topoLines.addObject('EdgeSetTopologyContainer', name='meshLines')
    topoLines.addObject('EdgeSetTopologyModifier', name='Modifier')
    topoLines.addObject('EdgeSetGeometryAlgorithms', name='GeomAlgo', template='Rigid3d')
    topoLines.addObject('MechanicalObject', name='dofTopo2', template='Rigid3d')



    BeamMechanics = rootNode.addChild('BeamModel')
    BeamMechanics.addObject('EulerImplicitSolver', rayleighStiffness=0.2, rayleighMass=0.1)
    BeamMechanics.addObject('BTDLinearSolver', verification=False, subpartSolve=False, verbose=False)
    BeamMechanics.addObject('RegularGridTopology', name='MeshLines', 
                                    nx=61, ny=1, nz=1,
                                    xmax=200.0, xmin=0.0, ymin=0, ymax=0, zmax=0, zmin=0,
                                    p0=[0,0,0])
    BeamMechanics.addObject('MechanicalObject', showIndices=False, name='DOFs', template='Rigid3d', ry=-90)
    BeamMechanics.addObject('WireBeamInterpolation', name='BeamInterpolation', WireRestShape='@../EdgeTopology/BeamRestShape', printLog=False)
    BeamMechanics.addObject('AdaptiveBeamForceFieldAndMass', name='BeamForceField', massDensity=0.00000155, interpolation='@BeamInterpolation')
    BeamMechanics.addObject('InterventionalRadiologyController', name='DeployController', template='Rigid3d', instruments='BeamInterpolation', 
                                    topology="@MeshLines", startingPos=[0, 0, 0, 0, 0, 0, 1], xtip=[0], printLog=True, 
                                    rotationInstrument=[0], step=5., speed=5., 
                                    listening=True, controlledInstrument=0)
    BeamMechanics.addObject('LinearSolverConstraintCorrection', wire_optimization='true', printLog=False)
    BeamMechanics.addObject('FixedProjectiveConstraint', indices=0, name='FixedConstraint')
    BeamMechanics.addObject('RestShapeSpringsForceField', points='@DeployController.indexFirstNode', angularStiffness=1e8, stiffness=1e8)



    BeamVisu = BeamMechanics.addChild('VisualModel')
    BeamVisu.addObject('MeshSTLLoader', name='loader', filename='innertube.stl')
    BeamVisu.addObject('OglModel', name='Visual', src='@loader', color=[0.7, 0.7, 0.7, 1.0])
    BeamVisu.addObject('AdaptiveBeamMapping', 
                       input='@../DOFs', 
                       output='@Visual',
                       interpolation='@../BeamInterpolation',
                       useCurvAbs=True)



    BeamCollis = BeamMechanics.addChild('CollisionModel')
    BeamCollis.activated = True
    BeamCollis.addObject('EdgeSetTopologyContainer', name='collisEdgeSet')
    BeamCollis.addObject('EdgeSetTopologyModifier', name='colliseEdgeModifier')
    BeamCollis.addObject('MechanicalObject', name='CollisionDOFs')
    BeamCollis.addObject('MultiAdaptiveBeamMapping', controller='../DeployController', useCurvAbs=True, printLog=False, name='collisMap')
    BeamCollis.addObject('LineCollisionModel', proximity=0.0)
    BeamCollis.addObject('PointCollisionModel', proximity=0.0)


def main():
    import SofaRuntime
    import Sofa.Gui

    root = Sofa.Core.Node('root')
    createScene(root)
    Sofa.Simulation.init(root)

    Sofa.Gui.GUIManager.Init('myscene', 'qglviewer')
    Sofa.Gui.GUIManager.createGUI(root, __file__)
    Sofa.Gui.GUIManager.SetDimension(1080, 1080)
    Sofa.Gui.GUIManager.MainLoop(root)
    Sofa.Gui.GUIManager.closeGUI()


# Function used only if this script is called from a python environment
if __name__ == '__main__':
    main()
