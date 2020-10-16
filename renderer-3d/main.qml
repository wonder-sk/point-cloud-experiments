import QtQuick 2.1 as QQ2
import Qt3D.Core 2.0
import Qt3D.Render 2.0
import Qt3D.Input 2.0
import Qt3D.Extras 2.0
import Qt3D.Logic 2.0
import QtQml 2.0

Entity {

    components: [
        rendSettings,
        inputSettings,
        frameAction
    ]

    InputSettings { id: inputSettings }

    RenderSettings {
        id: rendSettings
        activeFrameGraph: RenderSurfaceSelector {
            Viewport {
                normalizedRect: Qt.rect(0,0,1,1)
                CameraSelector {
                    camera: camera
                    ClearBuffers {
                        buffers: ClearBuffers.ColorDepthBuffer
                    }
                }
            }
        }

    }

    FrameAction {
        id: frameAction
        onTriggered: frames++
        property real frames: 0
        property real secs: 0
    }

    Timer {
        interval: 1000
        running: true
        repeat: true
        onTriggered: {
            _window.setTitle("FPS: " + frameAction.frames)
            frameAction.frames = 0
            frameAction.secs++
        }
    }

    Camera {
        id: camera
        projectionType: CameraLens.PerspectiveProjection
        fieldOfView: 45
        aspectRatio: _window.width / _window.height
        nearPlane: 0.1
        farPlane: 100.0
        position: Qt.vector3d(0.0, 10.0, 20.0)
        viewCenter: Qt.vector3d(0.0, 0.0, 0.0)
        upVector: Qt.vector3d(0.0, 1.0, 0.0)
    }

    FirstPersonCameraController {
        camera: camera
        lookSpeed: 500
    }

    Entity {
        GeometryRenderer {
            id: gr
            primitiveType: GeometryRenderer.Points
            //vertexCount: 156000 //10000000
            vertexCount: _bbg.count
            geometry: _bbg
        }

        Transform {
            id: trr
            translation: Qt.vector3d(0,1.5,0)
        }

        Material {
            id: grm

            effect: Effect {
                techniques: Technique {
                    graphicsApiFilter { api: GraphicsApiFilter.OpenGL; profile: GraphicsApiFilter.CoreProfile; majorVersion: 3; minorVersion: 1 }
                    renderPasses: [
                        RenderPass {
                            shaderProgram: ShaderProgram {
                                vertexShaderCode: loadSource("qrc:/shaders/pointcloud.vert")
                                fragmentShaderCode: loadSource("qrc:/shaders/pointcloud.frag")
                            }
                            renderStates: [
                                //supported since OpenGL 3.2
                                //PointSize { sizeMode: PointSize.Fixed; value: 5.0 }
                                PointSize { sizeMode: PointSize.Programmable }
                            ]
                        }
                    ]
                }
            }
        }

        components:  [ gr, grm, trr ]
    }

}
