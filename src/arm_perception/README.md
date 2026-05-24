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
                                  
