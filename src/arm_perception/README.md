  ## Webcam Demo                 
                                                                                                                                                       
  Runs on any Mac or Linux machine with a webcam. Uses a pre-trained YOLOv8 model to detect coffee-related objects in real time.                       
                                                                                                                                                       
  **Requirements:** Python 3.8+, yolov8                                                                                                             
                  
  ```bash                                                                                                                                              
  pip install ultralytics opencv-python
  python scripts/webcam_demo.py
```
                                                                                                                                                       
  YOLOv8 weights (yolov8n.pt) download automatically on first run (~6 MB).                                                                             
                                                                                                                                                       
  Detected objects: cup, bottle, spoon, bowl, knife, fork                                                                                              
                  
  Controls:                                                                                                                                            
  - q — quit      
  - s — save screenshot
  - d — toggle info overlay


  ## Perception Launch with a RealSense Camera

  Runs on Linux with a RealSense RGB and depth camera. See Section Using WSL2 for Windows.

  To launch visualization (Rviz), run this inside the container:

  ```bash 

  source /opt/ros/humble/setup.bash  
  rviz2                                        
```

  ## Using WSL2

  Can work on WSL2

  Make sure that WSL2 can see USB attachments (use USBIPD).
                                  
