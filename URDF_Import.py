
import logging
import os
from typing import Annotated, Optional

import vtk

import slicer
from slicer.i18n import tr as _
from slicer.i18n import translate
from slicer.ScriptedLoadableModule import *
from slicer.util import VTKObservationMixin
from slicer.parameterNodeWrapper import (
    parameterNodeWrapper,
    WithinRange,
)

from slicer import vtkMRMLScalarVolumeNode


#
# URDF_Import
#
#TODO: ask about incorporating URDF to XACRO with someone else's script - what credit is necessary
#TODO: go through xacro2urdf code and see how to remove the package//: in the new files - maybe look for filename
#TODO: ask about incorporating XACRO to URDF script - what credit is required

class URDF_Import(ScriptedLoadableModule):
    """Uses ScriptedLoadableModule base class, available at:
    https://github.com/Slicer/Slicer/blob/main/Base/Python/slicer/ScriptedLoadableModule.py
    """

    def __init__(self, parent):
        ScriptedLoadableModule.__init__(self, parent)
        self.parent.title = _("URDF_Import")  # TODO: make this more human readable by adding spaces
        # TODO: set categories (folders where the module shows up in the module selector)
        self.parent.categories = [translate("qSlicerAbstractCoreModule", "Examples")]
        self.parent.dependencies = []  # TODO: add here list of module names that this module requires
        self.parent.contributors = ["John Doe (AnyWare Corp.)"]  # TODO: replace with "Firstname Lastname (Organization)"
        # TODO: update with short description of the module and a link to online module documentation
        # _() function marks text as translatable to other languages
        self.parent.helpText = _("""
This is an example of scripted loadable module bundled in an extension.
See more information in <a href="https://github.com/organization/projectname#URDF_Import">module documentation</a>.
""")
        # TODO: replace with organization, grant and thanks
        self.parent.acknowledgementText = _("""
This file was originally developed by Jean-Christophe Fillion-Robin, Kitware Inc., Andras Lasso, PerkLab,
and Steve Pieper, Isomics, Inc. and was partially funded by NIH grant 3P41RR013218-12S1.
""")

        # Additional initialization step after application startup is complete
    #    slicer.app.connect("startupCompleted()", registerSampleData)



    


def connectNodes(nodes, scaleTrans):
        robotToWorldTransformNode = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLTransformNode", "Robot")
        robotToWorldTransform = vtk.vtkTransform()
        if scaleTrans:
            robotToWorldTransform.Scale(1000, 1000, 1000)  # convert from meters (URDF) to millimeters (Slicer)
        robotToWorldTransformNode.SetMatrixTransformToParent(robotToWorldTransform.GetMatrix())
        for nodeName in nodes:
            if nodes[nodeName]["type"] == "link":
                node = nodes[nodeName]["model"]
            elif nodes[nodeName]["type"] == "joint" or nodes[nodeName]["type"] == "transform":
                node = nodes[nodeName]["transform"]
            if not node.GetParentTransformNode():
                node.SetAndObserveTransformNodeID(robotToWorldTransformNode.GetID())

def makeNodeHierarchy(nodes, robot):
    for joint in robot.findall("joint"):
        name = joint.get("name")

        parentName = joint.find("parent").get("link")
        if parentName:
            parent = nodes[parentName]
            if parent["type"] != "link":
                raise ValueError(f"Parent of joint {name} is not a link")
            jointToParentTransformNode = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLTransformNode", f"{name} to {parentName}")
            nodes[jointToParentTransformNode.GetName()] = { "type": "transform", "transform": jointToParentTransformNode}
            jointToParentTransformNode.SetAndObserveTransformNodeID(parent["model"].GetTransformNodeID())
            # <origin rpy="-1.57079632679 0 0" xyz="0 0 0"/>
            transformToParent = vtk.vtkTransform()
            rpy = [vtk.vtkMath.DegreesFromRadians(float(x)) for x in joint.find("origin").get("rpy").split()]  
            xyz = [float(x) for x in joint.find("origin").get("xyz").split()]
            myXYZ = [xyz[0], xyz[1], xyz[2]]
            transformToParent.Translate(myXYZ)
            transformToParent.RotateX(rpy[0])
            transformToParent.RotateY(rpy[1])
            transformToParent.RotateZ(rpy[2])
            jointToParentTransformNode.SetMatrixTransformToParent(transformToParent.GetMatrix())
            nodes[name]["transform"].SetAndObserveTransformNodeID(jointToParentTransformNode.GetID())
        
        # iterate through all children
        for child in joint.findall("child"):
            childName = child.get("link")
            child = nodes[childName]
            if child["type"] != "link":
                raise ValueError(f"Child of joint {name} is not a link")
            childModelNode = child["model"]
            childModelNode.SetAndObserveTransformNodeID(nodes[name]["transform"].GetID())

def setLimits(link):
    if link.find("limit").get("lower") != None:
        lowerLimit = link.find("limit").get("lower")
    if link.find("limit").get("upper") != None:
        upperLimit = link.find("limit").get("upper")

#def xacroToUrdf():
    #TODO: incorporate code from xacro2urdf github possibly 

#
# URDF_ImportParameterNode
#


@parameterNodeWrapper
class URDF_ImportParameterNode:
    """
    The parameters needed by module.

    inputVolume - The volume to threshold.
    imageThreshold - The value at which to threshold the input volume.
    invertThreshold - If true, will invert the threshold.
    thresholdedVolume - The output volume that will contain the thresholded volume.
    invertedVolume - The output volume that will contain the inverted thresholded volume.
    """

    inputVolume: vtkMRMLScalarVolumeNode
    imageThreshold: Annotated[float, WithinRange(-100, 500)] = 100
    invertThreshold: bool = False
    thresholdedVolume: vtkMRMLScalarVolumeNode
    invertedVolume: vtkMRMLScalarVolumeNode


#
# URDF_ImportWidget
#


class URDF_ImportWidget(ScriptedLoadableModuleWidget, VTKObservationMixin):
    """Uses ScriptedLoadableModuleWidget base class, available at:
    https://github.com/Slicer/Slicer/blob/main/Base/Python/slicer/ScriptedLoadableModule.py
    """

    def __init__(self, parent=None) -> None:
        """Called when the user opens the module the first time and the widget is initialized."""
        ScriptedLoadableModuleWidget.__init__(self, parent)
        VTKObservationMixin.__init__(self)  # needed for parameter node observation
        self.logic = None
        self._parameterNode = None
        self._parameterNodeGuiTag = None

    def setup(self) -> None:
        """Called when the user opens the module the first time and the widget is initialized."""
        ScriptedLoadableModuleWidget.setup(self)

        # Load widget from .ui file (created by Qt Designer).
        # Additional widgets can be instantiated manually and added to self.layout.
        uiWidget = slicer.util.loadUI(self.resourcePath("UI/URDF_Import.ui"))
        self.layout.addWidget(uiWidget)
        self.ui = slicer.util.childWidgetVariables(uiWidget)

        # Set scene in MRML widgets. Make sure that in Qt designer the top-level qMRMLWidget's
        # "mrmlSceneChanged(vtkMRMLScene*)" signal in is connected to each MRML widget's.
        # "setMRMLScene(vtkMRMLScene*)" slot.
        uiWidget.setMRMLScene(slicer.mrmlScene)

        # Create logic class. Logic implements all computations that should be possible to run
        # in batch mode, without a graphical user interface.
        self.logic = URDF_ImportLogic()

        # Connections

        # These connections ensure that we update parameter node when scene is closed
        self.addObserver(slicer.mrmlScene, slicer.mrmlScene.StartCloseEvent, self.onSceneStartClose)
        self.addObserver(slicer.mrmlScene, slicer.mrmlScene.EndCloseEvent, self.onSceneEndClose)

        # Buttons
        self.ui.applyButton.connect("clicked(bool)", self.onApplyButton)
        self.ui.clearButton.connect("clicked(bool)", self.onClearButton)
        

        # Make sure parameter node is initialized (needed for module reload)
        self.initializeParameterNode()

    def cleanup(self) -> None:
        """Called when the application closes and the module widget is destroyed."""
        self.removeObservers()

    def enter(self) -> None:
        """Called each time the user opens this module."""
        # Make sure parameter node exists and observed
        self.initializeParameterNode()

    def exit(self) -> None:
        """Called each time the user opens a different module."""
        # Do not react to parameter node changes (GUI will be updated when the user enters into the module)
        if self._parameterNode:
            self._parameterNode.disconnectGui(self._parameterNodeGuiTag)
            self._parameterNodeGuiTag = None
            self.removeObserver(self._parameterNode, vtk.vtkCommand.ModifiedEvent, self._checkCanApply)

    def onSceneStartClose(self, caller, event) -> None:
        """Called just before the scene is closed."""
        # Parameter node will be reset, do not use it anymore
        self.setParameterNode(None)

    def onSceneEndClose(self, caller, event) -> None:
        """Called just after the scene is closed."""
        # If this module is shown while the scene is closed then recreate a new parameter node immediately
        if self.parent.isEntered:
            self.initializeParameterNode()

    def initializeParameterNode(self) -> None:
        """Ensure parameter node exists and observed."""
        # Parameter node stores all user choices in parameter values, node selections, etc.
        # so that when the scene is saved and reloaded, these settings are restored.

        self.setParameterNode(self.logic.getParameterNode())

        # Select default input nodes if nothing is selected yet to save a few clicks for the user
        if not self._parameterNode.inputVolume:
            firstVolumeNode = slicer.mrmlScene.GetFirstNodeByClass("vtkMRMLScalarVolumeNode")
            if firstVolumeNode:
                self._parameterNode.inputVolume = firstVolumeNode

    def setParameterNode(self, inputParameterNode: Optional[URDF_ImportParameterNode]) -> None:
        """
        Set and observe parameter node.
        Observation is needed because when the parameter node is changed then the GUI must be updated immediately.
        """

        if self._parameterNode:
            self._parameterNode.disconnectGui(self._parameterNodeGuiTag)
            self.removeObserver(self._parameterNode, vtk.vtkCommand.ModifiedEvent, self._checkCanApply)
        self._parameterNode = inputParameterNode
        if self._parameterNode:
            # Note: in the .ui file, a Qt dynamic property called "SlicerParameterName" is set on each
            # ui element that needs connection.
            self._parameterNodeGuiTag = self._parameterNode.connectGui(self.ui)
            self.addObserver(self._parameterNode, vtk.vtkCommand.ModifiedEvent, self._checkCanApply)
            self._checkCanApply()

    def _checkCanApply(self, caller=None, event=None) -> None:
        self.ui.applyButton.enabled = True
        #if self._parameterNode and self._parameterNode.inputVolume and self._parameterNode.thresholdedVolume:
            #self.ui.applyButton.toolTip = _("Compute output volume")
            #self.ui.applyButton.enabled = True
        #else:
            #self.ui.applyButton.toolTip = _("Select input and output volume nodes")
            #self.ui.applyButton.enabled = False

    def onClearButton(self) -> None:
        slicer.mrmlScene.Clear()

    def onApplyButton(self) -> None:

        """Run processing when user clicks "Apply" button."""
       # import SampleData
        # Gets paths for the robot and the directory of mesh files from user input
        robotPath = self.ui.robotFilePath.currentPath
        meshFolder = self.ui.meshesDirectoryButton.directory
        scaleIsM = self.ui.scaleRobotFileM.checked
        print(robotPath)
        print(meshFolder)
        #TODO: 
        """ TODO: something like
        if(robotPath ending is xacro):
            use script to change to urdf
            also in script change the filename thing
        """
        #downloadedFolder = SampleData.downloadFromURL(
            #fileNames="RobotDescription.zip",
            #uris="https://github.com/justagist/franka_panda_description/archive/refs/heads/master.zip")[0] #put the r2d2 file in here
        #rootPath = downloadedFolder + "/franka_panda_description-master" # root path for models
        #urdfFilePath = paths
        #rootPath + "/robots/panda_arm.urdf"
        # Parse robot description file   
        import xml.etree.ElementTree as ET
        # Parse XML data from a file
        tree = ET.parse(robotPath)
        print("tree" + tree) #TODO: check how this works for path file thing
        robot = tree.getroot()
        if robot.tag != "robot":
            raise ValueError("Invalid URDF file")
        else:
            print("robot successful!")
        
        nodes = {}
        for link in robot:
            name = link.get("name")
            if link.tag == "link":
                try: 
                    stlFilePath = meshFolder + '/' + link.find('visual').find('geometry').find('mesh').attrib["filename"]
                    print(stlFilePath)
                    #rootPath + "/" + link.find('collision').find('geometry').find('mesh').attrib["filename"]
                    print("mesh found")
                    # Use RAS coordinate system to avoid model conversion from LPS to RAS (we can transform the entire robot as a whole later if needed)
                    modelNode = slicer.modules.models.logic().AddModel(stlFilePath, slicer.vtkMRMLStorageNode.CoordinateSystemRAS)
                except:
                    # No mesh found, add a sphere
                    print("sphere in use")
                    sphere = vtk.vtkSphereSource()
                    sphere.SetRadius(0.01)
                    modelNode = slicer.modules.models.logic().AddModel(sphere.GetOutputPort())
                modelNode.SetName(name)
                nodes[name] = { "type": "link", "model": modelNode}
            elif link.tag == "joint":
                jointTransformNode = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLTransformNode", name)
                nodes[name] = { "type": "joint", "transform": jointTransformNode}
                if link.get("type") == "fixed":
                    # do not create a display node, the transform does not have to be editable
                    pass
                else:
                    # make the transform interactively editable in 3D views
                    jointTransformNode.CreateDefaultDisplayNodes()
                    displayNode = jointTransformNode.GetDisplayNode()
                    displayNode.SetEditorVisibility(True)
                    displayNode.SetEditorSliceIntersectionVisibility(False)
                    displayNode.SetEditorTranslationEnabled(False)
                    if not link.get("type") == "floating":
                        if(link.find("axis") == None):
                            axis = [1, 0, 0]
                        else:
                            axis = [float(x) for x in link.find("axis").get("xyz").split()]
                    if link.get("type") == "revolute":
                        #<axis xyz="0 0 1"/>
                        #rotationAxis = [float(x) for x in link.find("axis").get("xyz").split()]
                        setLimits(link)

                        #what to do with these limits?

                        if axis == [1, 0, 0] or axis == [-1, 0, 0]:
                            displayNode.SetRotationHandleComponentVisibility3D(True, False, False, False)
                        elif axis == [0, 1, 0] or axis == [0, -1, 0]:
                            displayNode.SetRotationHandleComponentVisibility3D(False, True, False, False)
                        elif axis == [0, 0, 1] or axis == [0, 0, -1]:
                            displayNode.SetRotationHandleComponentVisibility3D(False, False, True, False)
                        else:
                            raise ValueError(f"Unsupported rotation axis {axis}")
                    elif link.get("type") == "continuous":
                        
                        if axis == [1, 0, 0] or axis == [-1, 0, 0]:
                            displayNode.SetRotationHandleComponentVisibility3D(True, False, False, False)
                        elif axis == [0, 1, 0] or axis == [0, -1, 0]:
                            displayNode.SetRotationHandleComponentVisibility3D(False, True, False, False)
                        elif axis == [0, 0, 1] or axis == [0, 0, -1]:
                            displayNode.SetRotationHandleComponentVisibility3D(False, False, True, False)
                        else:
                            raise ValueError(f"Unsupported continuous axis {axis}")
                    elif link.get("type") == "prismatic":
                        # TODO: implement prismatic joint
                        displayNode.SetEditorTranslationEnabled(True)
                        displayNode.SetEditorRotationEnabled(False)
                        if axis == [1, 0, 0] or axis == [-1, 0, 0]:
                            displayNode.SetTranslationHandleComponentVisibility3D(True, False, False, False)
                        elif axis == [0, 1, 0] or axis == [0, -1, 0]:
                            displayNode.SetTranslationHandleComponentVisibility3D(False, True, False, False)
                        elif axis == [0, 0, 1] or axis == [0, 0, -1]:
                            displayNode.SetTranslationHandleComponentVisibility3D(False, False, True, False)
                        else:
                            raise ValueError(f"Unsupported prismatic axis {axis}")
                    elif link.get("type") == "floating":
                        # TODO: implement floating joint
                        displayNode.SetEditorTranslationEnabled(True)
                        displayNode.SetRotationHandleComponentVisibility3D(True, True, True, False) #check this last one - should it stay as false
                        displayNode.SetTranslationHandleComponentVisibility3D(True, True, True, False) #what is this last one if the first 3 are x, y, z
                    #elif link.get("type") == "planar":
                        # TODO: implement planar joint
                    else:
                        # TODO: implement translation and other joint types
                        raise ValueError(f"Unsupported joint type {link.get('type')}")
        makeNodeHierarchy(nodes, robot)
        connectNodes(nodes, scaleIsM)
                
        
            

#
# URDF_ImportThresholdLogic
#


class URDF_ImportLogic(ScriptedLoadableModuleLogic):
    """This class should implement all the actual
    computation done by your module.  The interface
    should be such that other python code can import
    this class and make use of the functionality without
    requiring an instance of the Widget.
    Uses ScriptedLoadableModuleLogic base class, available at:
    https://github.com/Slicer/Slicer/blob/main/Base/Python/slicer/ScriptedLoadableModule.py
    """

    def __init__(self) -> None:
        """Called when the logic class is instantiated. Can be used for initializing member variables."""
        ScriptedLoadableModuleLogic.__init__(self)

    def getParameterNode(self):
        return URDF_ImportParameterNode(super().getParameterNode())

    def process(self,
                inputVolume: vtkMRMLScalarVolumeNode,
                outputVolume: vtkMRMLScalarVolumeNode,
                imageThreshold: float,
                invert: bool = False,
                showResult: bool = True) -> None:
        """
        Run the processing algorithm.
        Can be used without GUI widget.
        :param inputVolume: volume to be thresholded
        :param outputVolume: thresholding result
        :param imageThreshold: values above/below this threshold will be set to 0
        :param invert: if True then values above the threshold will be set to 0, otherwise values below are set to 0
        :param showResult: show output volume in slice viewers
        """

        if not inputVolume or not outputVolume:
            raise ValueError("Input or output volume is invalid")

        import time

        startTime = time.time()
        logging.info("Processing started")

        # Compute the thresholded output volume using the "Threshold Scalar Volume" CLI module
        cliParams = {
            "InputVolume": inputVolume.GetID(),
            "OutputVolume": outputVolume.GetID(),
            "ThresholdValue": imageThreshold,
            "ThresholdType": "Above" if invert else "Below",
        }
        cliNode = slicer.cli.run(slicer.modules.thresholdscalarvolume, None, cliParams, wait_for_completion=True, update_display=showResult)
        # We don't need the CLI module node anymore, remove it to not clutter the scene with it
        slicer.mrmlScene.RemoveNode(cliNode)

        stopTime = time.time()
        logging.info(f"Processing completed in {stopTime-startTime:.2f} seconds")

    
	


#
# URDF_ImportTest
#


class URDF_ImportTest(ScriptedLoadableModuleTest):
    """
    This is the test case for your scripted module.
    Uses ScriptedLoadableModuleTest base class, available at:
    https://github.com/Slicer/Slicer/blob/main/Base/Python/slicer/ScriptedLoadableModule.py
    """

    def setUp(self):
        """Do whatever is needed to reset the state - typically a scene clear will be enough."""
        slicer.mrmlScene.Clear()

    def runTest(self):
        """Run as few or as many tests as needed here."""
        self.setUp()
        self.test_URDF_Import1()

    def test_URDF_Import1(self):
        """Ideally you should have several levels of tests.  At the lowest level
        tests should exercise the functionality of the logic with different inputs
        (both valid and invalid).  At higher levels your tests should emulate the
        way the user would interact with your code and confirm that it still works
        the way you intended.
        One of the most important features of the tests is that it should alert other
        developers when their changes will have an impact on the behavior of your
        module.  For example, if a developer removes a feature that you depend on,
        your test should break so they know that the feature is needed.
        """

        self.delayDisplay("Starting the test")

        # Get/create input data

        import SampleData

        registerSampleData()
        inputVolume = SampleData.downloadSample("URDF_Import1")
        self.delayDisplay("Loaded test data set")

        inputScalarRange = inputVolume.GetImageData().GetScalarRange()
        self.assertEqual(inputScalarRange[0], 0)
        self.assertEqual(inputScalarRange[1], 695)

        outputVolume = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLScalarVolumeNode")
        threshold = 100

        # Test the module logic

        logic = URDF_ImportLogic()

        # Test algorithm with non-inverted threshold
        logic.process(inputVolume, outputVolume, threshold, True)
        outputScalarRange = outputVolume.GetImageData().GetScalarRange()
        self.assertEqual(outputScalarRange[0], inputScalarRange[0])
        self.assertEqual(outputScalarRange[1], threshold)

        # Test algorithm with inverted threshold
        logic.process(inputVolume, outputVolume, threshold, False)
        outputScalarRange = outputVolume.GetImageData().GetScalarRange()
        self.assertEqual(outputScalarRange[0], inputScalarRange[0])
        self.assertEqual(outputScalarRange[1], inputScalarRange[1])

        self.delayDisplay("Test passed")