import fcntl, os, sys
from pathlib import Path
root=Path(__file__).resolve().parent
lock=(root/'live.lock').open('a')
try: fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
except BlockingIOError: raise SystemExit('A live candidate from this bundle is already running')
(root/'live.pid').write_text(str(os.getpid())+'\n')
sys.path.insert(0,str(root/'package'))
from mdp_perception.live_perception_node import main
try: main()
finally: (root/'live.pid').unlink(missing_ok=True)
