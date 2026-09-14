import json,time
from pathlib import Path
import cv2,rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter,ParameterValue,ParameterType
root=Path('/home/mdp/mdp-cv/reports/medium-post-training-20260914/replay-final')
root.mkdir(parents=True,exist_ok=True)
images=Path('/home/mdp/mdp-cv/reports/medium-post-training-20260914/cases')
frames={k:cv2.imread(str(images/name)) for k,name in {'F':'letter-f.png','background':'background.png','1':'digit1.png'}.items()}
assert all(f is not None for f in frames.values())
rclpy.init(); node=Node('medium_fixture_replay'); bridge=CvBridge()
pub=node.create_publisher(Image,'/test/medium/fixtures/image_raw',10)
received=[]; current=['F']; phase=['startup']; started=time.monotonic()
def on_result(msg):
    event={'message':msg.data,'phase':phase[0],'elapsed_seconds':time.monotonic()-started}
    received.append(event); print(json.dumps(event),flush=True)
sub=node.create_subscription(String,'/test/medium/fixtures/target',on_result,10)
def publish():
    msg=bridge.cv2_to_imgmsg(frames[current[0]],encoding='bgr8')
    msg.header.stamp=node.get_clock().now().to_msg(); msg.header.frame_id='recorded_fixture'
    pub.publish(msg)
timer=node.create_timer(0.1,publish)
def spin_for(seconds):
    deadline=time.monotonic()+seconds
    while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=0.1)
def wait_count(count,timeout=90):
    deadline=time.monotonic()+timeout
    while len(received)<count and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=0.1)
    assert len(received)==count,f'Expected {count} messages, got {received}'
passed=False
try:
    phase[0]='confirm_F'; wait_count(1); assert received[-1]['message']=='1,25'
    phase[0]='hold_F'; spin_for(12); assert len(received)==1,'Duplicate spam while F held'
    phase[0]='background'; current[0]='background'; spin_for(20); assert len(received)==1,'Background false publication'
    phase[0]='F_reappears'; current[0]='F'; wait_count(2); assert received[-1]['message']=='1,25'
    phase[0]='switch_to_digit1'; current[0]='1'; wait_count(3); assert received[-1]['message']=='1,11'
    phase[0]='change_obstacle'
    client=node.create_client(SetParameters,'/perception_medium_candidate/set_parameters')
    assert client.wait_for_service(timeout_sec=5)
    req=SetParameters.Request(); req.parameters=[Parameter(name='obstacle_id',value=ParameterValue(type=ParameterType.PARAMETER_INTEGER,integer_value=2))]
    future=client.call_async(req)
    deadline=time.monotonic()+5
    while not future.done() and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=0.1)
    assert future.done() and future.result().results[0].successful
    wait_count(4); assert received[-1]['message']=='2,11'
    phase[0]='hold_digit1'; spin_for(12); assert len(received)==4
    passed=True
finally:
    record={'passed':passed,'events':received,'checks':['automatic publishing without sample service','three processed frame confirmation','held-symbol duplicate suppression','background rejection','rearm after absence','switch symbol','obstacle change resets confirmation'],'input':'Previously captured Pi camera images replayed at 10 FPS; real YOLOv8m ONNX inference on Pi; no new physical observations'}
    (root/'replay-result.json').write_text(json.dumps(record,indent=2)+'\n')
    print(json.dumps(record,indent=2),flush=True)
    timer.cancel(); node.destroy_node()
    if rclpy.ok():rclpy.shutdown()
