//using System.Windows;
//using System.Windows.Media.Media3D;
//using System.Windows.Media;
//using HelixToolkit.Wpf;
//using System.IO;

//namespace WPFRobot
//{
//    public partial class MainWindow : Window
//    {
//        public MainWindow()
//        {
//            InitializeComponent();
//            //DrawLine();
//            ShowDefaultAxisArrows();
//            LoadStlFile(@"C:\Users\Weifeng Huang\Desktop\stl\Assembly v17_Assembly v17_J1J2J3Mount v25_1_Body1.OBJ");
//            //LoadStlFile(@"C:\\Users\\Weifeng Huang\\Desktop\\WHIOT\\Reservoom\\WPFRobot\\3DModels\\J1J2J3Mount v252.stl");
//        }

//        private void LoadStlFile(string filePath)
//        {
//            if (File.Exists(filePath))
//            {
//                var stlReader = new StLReader();
//                var mesh = stlReader.Read(filePath);

//                // Calculate the bounding box of the mesh to find its center
//                var boundingBox = mesh.Bounds;

//                var center = boundingBox.GetCenter();

//                // Create a translation transform to move the model to the origin
//                //var translation = new TranslateTransform3D(-center.X, -center.Y, -center.Z);

//                // Create a ModelVisual3D and apply the translation transform
//                var modelVisual = new ModelVisual3D
//                {
//                    Content = mesh,
//                    //Transform = translation // Apply the translation transform

//                };

//                helixViewport.Children.Add(modelVisual);
//            }
//            else
//            {
//                MessageBox.Show("STL file not found: " + filePath);
//            }
//        }

//        private void ShowDefaultAxisArrows()
//        {
//            var coordinateSystem = new CoordinateSystemVisual3D
//            {
//                ArrowLengths = 10
//            };

//            helixViewport.Children.Add(coordinateSystem);
//        }
//    }
//}


using System.Windows;
using System.Windows.Media.Media3D;
using System.Windows.Media;
using Newtonsoft.Json;

using HelixToolkit.Wpf; 
using System.IO;
using NumSharp;

using uPLibrary.Networking.M2Mqtt.Messages;
using uPLibrary.Networking.M2Mqtt;
using System.Globalization;
using System.Numerics;


namespace WPFRobot
{
    public partial class MainWindow : Window
    {
        private RotateTransform3D rotateTransform;
        private TranslateTransform3D translateTransform;
        private Thread animationThread;

        private AxisAngleRotation3D rotation; // Field to store the rotation object for later updates

        private Matrix3D matrix;
        //private MatrixTransform3D tansformation;

        private 

        GeometryModel3D boxModel;
        GeometryModel3D box1Model;
        GeometryModel3D box2Model;
        GeometryModel3D box3Model;

        //Transform3D initBox2Tf;

        
        //NDArray box2InitalCenter ;
        //NDArray box2jointToCenter;
        NDArray box2JointCenter;


        NDArray box3JointCenter;


        RobotMqtt robotMqtt;

        List<NDArray> tmInitCurrBaseJoints = new List<NDArray>();
        public MainWindow()
        {
            robotMqtt = new RobotMqtt();
            robotMqtt.Init();
            robotMqtt.Subscribe("tmInitCurrBaseJoint");
            robotMqtt.client.MqttMsgPublishReceived += UpdateMqttMsg;
            

            InitializeComponent();
      

            //LoadObjFile(@"C:\Users\Weifeng Huang\Desktop\stl\Assembly v17_Assembly v17_J2Pulley T160 v22_1_Body1.OBJ"); // Replace with your OBJ file path
            //LoadObjFile(@"C:\Users\Weifeng Huang\Desktop\stl\Assembly v17_Assembly v17_J1J2J3Mount v25_1_Body1.OBJ"); // Replace with your OBJ file path
            ShowDefaultAxisArrows();
            //AddRotatingAndTranslatingBlock();
            AddTransformedBox();


        }

        //        private void ShowDefaultAxisArrows()
        //        {
        //            var coordinateSystem = new CoordinateSystemVisual3D
        //            {
        //                ArrowLengths = 10
        //            };

        //            helixViewport.Children.Add(coordinateSystem);
        //        }

        private void UpdateMqttMsg(object sender,MqttMsgPublishEventArgs e)
        {
            var msg = System.Text.Encoding.Default.GetString(e.Message).Split(':');
            if (msg[0] == "tmInitCurrBaseJoint")
            {
                tmInitCurrBaseJoints = ConvertTMBaseJioint( msg[1]);
                //var matrix2 = ConvertToMatrix3D(sss);
            }
        }

        static List<NDArray> ConvertTMBaseJioint(string input)
        {
            var matrixList = JsonConvert.DeserializeObject<List<List<List<double>>>>(input);

            // Convert the List<List<List<double>>> into List<double[,]>
            List<NDArray> matrices = new List<NDArray>();

            foreach (var matrix in matrixList)
            {
                double[,] matrixArray = new double[4, 4];
                for (int i = 0; i < 4; i++)
                {
                    for (int j = 0; j < 4; j++)
                    {
                        matrixArray[i, j] = matrix[i][j];
                    }
                }
                matrices.Add(new NDArray(matrixArray));
            }
            return matrices;
        }

        private void AddTransformedBox()
        {
           
            // Create the first box
            var meshBuilder = new MeshBuilder();
            meshBuilder.AddBox(new Point3D(0, 0, 25), 5, 5, 50); // Center at origin, 5x5x50 size
            var box1Mesh = meshBuilder.ToMesh();
            var box1Material = MaterialHelper.CreateMaterial(Colors.CornflowerBlue); // Color for the first box
            box1Model = new GeometryModel3D
            {
                Geometry = box1Mesh,
                Material = box1Material,
                BackMaterial = box1Material
            };
            

            // Create the second box
            meshBuilder = new MeshBuilder(); // Reset the mesh builder
            meshBuilder.AddBox(new Point3D(25, 0, 50), 50, 5, 5); // 50x5x5 size
            var box2Mesh = meshBuilder.ToMesh();
            var box2Material = MaterialHelper.CreateMaterial(Colors.Red); // Color for the second box
            box2Model = new GeometryModel3D
            {
                Geometry = box2Mesh,
                Material = box2Material,
                BackMaterial = box2Material
            };

            //box2InitalCenter = np.array( box2Mesh.Bounds.GetCenter().X, box2Mesh.Bounds.GetCenter().Y, box2Mesh.Bounds.GetCenter().Z,1);
            box2JointCenter = np.array( new double[] { 0,0,50,1});
            //box2jointToCenter = box2InitalCenter- box2JointCenter;
            //initBox2Tf = box2Model.Transform;

            // Create the third box
            meshBuilder = new MeshBuilder(); // Reset the mesh builder
            meshBuilder.AddBox(new Point3D(75, 0, 50), 50, 5, 5); // 50x5x5 size
            var box3Mesh = meshBuilder.ToMesh();
            var box3Material = MaterialHelper.CreateMaterial(Colors.Green); // Color for the third box
            box3Model = new GeometryModel3D
            {
                Geometry = box3Mesh,
                Material = box3Material,
                BackMaterial = box3Material
            };

            box3JointCenter = np.array(new double[] { 50, 0, 50, 1 });

            var unityMatrix = new Matrix3D(
            1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1);

            //matrix =  ConvertToMatrix3D(transformationMatrix);

            var tansformation = new MatrixTransform3D(unityMatrix);

            //var rotateTransform = new RotateTransform3D(rotation);
            // Apply the rotation to the model
            //boxModel.Transform = rotateTransform;
            //boxModel.Transform = tansformation;

            box1Model.Transform = tansformation;
            box2Model.Transform = tansformation;
            box3Model.Transform = tansformation;

            //MatrixTransform3D tm = new MatrixTransform3D(matrix);

            //boxModel.Transform = tm;

            // Add the box to the viewport
            helixViewport.Children.Add(new ModelVisual3D { Content = box1Model });
            helixViewport.Children.Add(new ModelVisual3D { Content = box2Model });
            helixViewport.Children.Add(new ModelVisual3D { Content = box3Model });
        }


        private void Button_Click(object sender, RoutedEventArgs e)
        {
            Task.Run(() => {
                while (true)
                {

                    Thread.Sleep(10);

                    this.Dispatcher.Invoke(() => {
                        UpdateArm2();

                        NDArray tmInitCurrBaseJoint = tmInitCurrBaseJoints[2];
                        NDArray rotationMatrix = np.array(new double[,]  {
                            { tmInitCurrBaseJoint[0, 0].GetDouble(), tmInitCurrBaseJoint[0, 1].GetDouble(), tmInitCurrBaseJoint[0, 2].GetDouble(),0 },
                            { tmInitCurrBaseJoint[1, 0].GetDouble(), tmInitCurrBaseJoint[1, 1].GetDouble(), tmInitCurrBaseJoint[1, 2].GetDouble(),0 },
                            { tmInitCurrBaseJoint[2, 0].GetDouble(), tmInitCurrBaseJoint[2, 1].GetDouble(), tmInitCurrBaseJoint[2, 2].GetDouble(),0 },
                            { 0,0,0,1}
                          });


                        // move part to orgin
                        var tm = np.array(new double[,] {
                            { 1.0, 0.0, 0.0, -box3JointCenter[0].GetDouble() },  // x translation
                            { 0.0, 1.0, 0.0, -box3JointCenter[1].GetDouble() },  // y translation
                            { 0.0, 0.0, 1.0, -box3JointCenter[2].GetDouble()},  // z translation
                            { 0.0, 0.0, 0.0, 1.0 }             // Homogeneous coordinate
                        });


                        //// rotate part to around
                        tm = np.matmul(tmInitCurrBaseJoint, tm);
                        //var tlm1 = np.array(new double[,] {
                        //    { 1.0, 0.0, 0.0, 50},  // x translation
                        //    { 0.0, 1.0, 0.0,0 },  // y translation
                        //    { 0.0, 0.0, 1.0, 50},  // z translation
                        //    { 0.0, 0.0, 0.0, 1.0 }             // Homogeneous coordinate
                        //});
                        //tm = np.matmul(tlm1, tm);

                        var Dtm = ConvertNDToMatrix3D(tm);
                        box3Model.Transform = new MatrixTransform3D(Dtm);


                    });
                  
                }
            
            });

            //rotation.Angle += 0.01;

            
        }
        private void UpdateArm2()
        {
            NDArray tmInitCurrBaseJoint = tmInitCurrBaseJoints[1];
            NDArray rotationMatrix1 = np.array(new double[,]  {
                            { tmInitCurrBaseJoint[0, 0].GetDouble(), tmInitCurrBaseJoint[0, 1].GetDouble(), tmInitCurrBaseJoint[0, 2].GetDouble(),0 },
                            { tmInitCurrBaseJoint[1, 0].GetDouble(), tmInitCurrBaseJoint[1, 1].GetDouble(), tmInitCurrBaseJoint[1, 2].GetDouble(),0 },
                            { tmInitCurrBaseJoint[2, 0].GetDouble(), tmInitCurrBaseJoint[2, 1].GetDouble(), tmInitCurrBaseJoint[2, 2].GetDouble(),0 },
                            { 0,0,0,1}
                          });

            // move part to orgin
            var tm1 = np.array(new double[,] {
                            { 1.0, 0.0, 0.0, -box2JointCenter[0].GetDouble() },  // x translation
                            { 0.0, 1.0, 0.0, -box2JointCenter[1].GetDouble() },  // y translation
                            { 0.0, 0.0, 1.0, -box2JointCenter[2].GetDouble()},  // z translation
                            { 0.0, 0.0, 0.0, 1.0 }             // Homogeneous coordinate
                        });

            // rotate part to around
            tm1 = np.matmul(rotationMatrix1, tm1);
            var tlm1 = np.array(new double[,] {
                            { 1.0, 0.0, 0.0, 0},  // x translation
                            { 0.0, 1.0, 0.0,0 },  // y translation
                            { 0.0, 0.0, 1.0, 50},  // z translation
                            { 0.0, 0.0, 0.0, 1.0 }             // Homogeneous coordinate
                        });
            tm1 = np.matmul(tlm1, tm1);

            var Dtm1 = ConvertNDToMatrix3D(tm1);
            box2Model.Transform = new MatrixTransform3D(Dtm1);
        }
        public Matrix3D ConvertNDToMatrix3D(NDArray NDMatrix)
        {
            var nm = new Matrix3D(
                NDMatrix[0, 0].GetDouble(), NDMatrix[1, 0].GetDouble(), NDMatrix[2, 0].GetDouble(), NDMatrix[3, 0].GetDouble(), // First row
                NDMatrix[0, 1].GetDouble(), NDMatrix[1, 1].GetDouble(), NDMatrix[2, 1].GetDouble(), NDMatrix[3, 1].GetDouble(), // Second row
                NDMatrix[0, 2].GetDouble(), NDMatrix[1, 2].GetDouble(), NDMatrix[2, 2].GetDouble(), NDMatrix[3, 2].GetDouble(), // Third row
                NDMatrix[0, 3].GetDouble(), NDMatrix[1, 3].GetDouble(), NDMatrix[2, 3].GetDouble(), NDMatrix[3, 3].GetDouble()  // Fourth row
                );

            return nm;
        }


        public Matrix3D ConvertDoubleToMatrix3D(double[,] matrix)
        {
            var nm =  new Matrix3D(
                matrix[0, 0], matrix[1, 0], matrix[2, 0], matrix[3, 0], // First row
                matrix[0, 1], matrix[1, 1], matrix[2, 1], matrix[3, 1], // Second row
                matrix[0, 2], matrix[1, 2], matrix[2, 2], matrix[3, 2], // Third row
                matrix[0, 3], matrix[1, 3], matrix[2, 3], matrix[3, 3]  // Fourth row
            );

            return nm;
        }

        public double[,] ConvertNDToDouble(NDArray NDMatrix)
        {
            double[,] doubleMatrix = new double[4, 4];
            for (int i = 0; i < 4; i++)
            {
                for (int j = 0; j < 4; j++)
                {
                    doubleMatrix[i, j] = NDMatrix[i, j].GetDouble();
                }
            }
            return doubleMatrix;
        }

        //private void updateAnimation()
        //{

        //}

        //public void UpdateRotationAndTranslation(double newRotationAngle, double newTranslationY)
        //{
        //    // Stop animations if they are running
        //    rotateTransform.BeginAnimation(RotateTransform3D.RotationProperty, null);
        //    translateTransform.BeginAnimation(TranslateTransform3D.OffsetYProperty, null);

        //    // Apply new rotation and translation values
        //    var rotation = (AxisAngleRotation3D)rotateTransform.Rotation;
        //    rotation.Angle = newRotationAngle;
        //    translateTransform.OffsetY = newTranslationY;

        //    // Restart animations if needed
        //    StartCombinedAnimation();
        //}
        //private void StartCombinedAnimation()
        //{
        //    // Set up rotation animation
        //    var rotationAnimation = new DoubleAnimation
        //    {
        //        From = 0,
        //        To = 360,
        //        Duration = TimeSpan.FromSeconds(5),
        //        RepeatBehavior = RepeatBehavior.Forever
        //    };
        //    var rotation = (AxisAngleRotation3D)rotateTransform.Rotation;
        //    rotation.BeginAnimation(AxisAngleRotation3D.AngleProperty, rotationAnimation);

        //    // Set up translation animation
        //    var translationAnimation = new DoubleAnimation
        //    {
        //        From = 0,
        //        To = 2, // Moves 2 units up
        //        Duration = TimeSpan.FromSeconds(2.5),
        //        AutoReverse = true,
        //        RepeatBehavior = RepeatBehavior.Forever
        //    };
        //    translateTransform.BeginAnimation(TranslateTransform3D.OffsetYProperty, translationAnimation);
        //}


        //private void AddRotatingAndTranslatingBlock()
        //{
        //    // Create the geometry for the block
        //    var meshBuilder = new MeshBuilder();
        //    meshBuilder.AddBox(new Point3D(3, 0, 0), 6, 1, 1); // Center at origin, 1x1x1 size
        //    var blockMesh = meshBuilder.ToMesh();

        //    // Create the material
        //    var material = MaterialHelper.CreateMaterial(Colors.SkyBlue);

        //    // Create the Model3D for the block
        //    var blockModel = new GeometryModel3D
        //    {
        //        Geometry = blockMesh,
        //        Material = material,
        //        BackMaterial = material
        //    };

        //    // Define the rotation axis and offset for combined rotation and translation
        //    var rotationAxis = new Vector3D(0, 1, 0); // Rotate around Y-axis
        //    var rotationOffset = new Vector3D(0, 0, 0); // Offset axis by (1, 1, 0)

        //    // Initialize the rotation and translation transforms as fields
        //    rotateTransform = new RotateTransform3D(new AxisAngleRotation3D(rotationAxis, 0));
        //    translateTransform = new TranslateTransform3D(0, 0, 0);

        //    // Combine transforms in a Transform3DGroup
        //    var transformGroup = new Transform3DGroup();
        //    transformGroup.Children.Add(new TranslateTransform3D(-rotationOffset.X, -rotationOffset.Y, -rotationOffset.Z)); // Move to rotation axis
        //    transformGroup.Children.Add(rotateTransform); // Apply rotation
        //    transformGroup.Children.Add(new TranslateTransform3D(rotationOffset.X, rotationOffset.Y, rotationOffset.Z)); // Move back
        //    transformGroup.Children.Add(translateTransform); // Add translation for movement
        //    blockModel.Transform = transformGroup;

        //    // Add the block to the viewport
        //    helixViewport.Children.Add(new ModelVisual3D { Content = blockModel });

        //    // Start both rotation and translation animations at the same time
        //    StartCombinedAnimation();
        //}



        //// Method to manually update rotation and translation




        private void LoadObjFile(string filePath)
        {
            if (File.Exists(filePath))
            {
                var objReader = new ObjReader();
                var mesh = objReader.Read(filePath);

                // Calculate the bounding box of the mesh to find its center
                var boundingBox = mesh.Bounds;
                var center = boundingBox.GetCenter();

                // Create a translation transform to move the model to the origin
                //var translation = new TranslateTransform3D(-center.X, -center.Y, -center.Z);

                // Create a ModelVisual3D and apply the translation transform
                var modelVisual = new ModelVisual3D
                {
                    Content = mesh,
                    //Transform = translation // Apply the translation transform
                };

                helixViewport.Children.Add(modelVisual);
            }
            else
            {
                MessageBox.Show("OBJ file not found: " + filePath);
            }
        }

        private void ShowDefaultAxisArrows()
        {
            var coordinateSystem = new CoordinateSystemVisual3D
            {
                ArrowLengths = 30
            };

            helixViewport.Children.Add(coordinateSystem);
        }

       
    }
}
