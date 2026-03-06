import numpy as np
import mmap
import os
import time

class SHMSender:
    def __init__(self, size_mb=100):
        self.size = size_mb * 1024 * 1024
        self.filename = "/dev/shm/slam_buffer"
        
        # --- RECREATION FIX ---
        # Instead of os.remove (which breaks C++ connection), 
        # we open in a way that preserves the file if it exists.
        fd = os.open(self.filename, os.O_RDWR | os.O_CREAT)
        
        # Set the size only if it's new or different
        if os.path.getsize(self.filename) != self.size:
            os.ftruncate(fd, self.size)
        
        # Create mmap and then close the fd (mmap keeps it open internally)
        self.mm = mmap.mmap(fd, self.size)
        os.close(fd)

        # --- STALE DATA FIX ---
        # Immediately zero out the headers (first 12 bytes: num_pcd, num_traj, num_kf)
        # This tells the C++ visualizer to clear its screen on startup.
        self.mm.seek(0)
        self.mm.write(np.array([0, 0, 0], dtype=np.int32).tobytes())
        
        print(f"SHM Bridge Ready: {self.filename}")

    def send_data(self, pcd, trajectory, keyframe_matrix):
        # 1. Prepare binary blobs
        pcd_bin = pcd.astype(np.float32).tobytes()
        traj_bin = trajectory.astype(np.float32).tobytes()
        kf_bin = keyframe_matrix.astype(np.float32).tobytes(order='F')

        # 2. Write Headers (Offsets: 0, 4, 8)
        # We write these LAST or use a flag if we wanted to be perfectly atomic,
        # but for visualization, sequential writing is fine.
        header = np.array([len(pcd), len(trajectory), 1], dtype=np.int32)
        
        # 3. Write Data at strict offsets matching C++ logic
        # PCD starts at 100
        self.mm.seek(100)
        self.mm.write(pcd_bin)
        
        # Trajectory follows PCD
        self.mm.write(traj_bin)
        
        # Keyframe follows Trajectory
        self.mm.write(kf_bin)

        # Final step: Update headers at the very start
        self.mm.seek(0)
        self.mm.write(header.tobytes())
        
    def __del__(self):
        # Ensure stale data isn't left behind when the script closes
        if hasattr(self, 'mm'):
            self.mm.seek(0)
            self.mm.write(np.array([0, 0, 0], dtype=np.int32).tobytes())
            self.mm.close()

# --- Simulation Code ---
if __name__ == "__main__":
    sender = SHMSender(size_mb=100)
    trajectory = []
    angle = 0.0

    print("SHM Simulation started. C++ viz_util can start/stop independently.")

    try:
        while True:
            angle += 0.05
            x, z = 3.0 * np.cos(angle), 3.0 * np.sin(angle)
            curr_pos = np.array([x, 0.0, z], dtype=np.float32)
            
            trajectory.append(curr_pos)
            if len(trajectory) > 100: trajectory.pop(0)

            # 4x4 Homogeneous Matrix (Rotation + Translation)
            rot_y = np.array([
                [np.cos(angle),  0, np.sin(angle), x],
                [0,              1, 0,             0],
                [-np.sin(angle), 0, np.cos(angle), z],
                [0,              0, 0,             1]
            ], dtype=np.float32)

            # Generate Points
            num_points = 20000
            points = np.random.normal(curr_pos, 0.8, (num_points, 3))
            colors = np.zeros((num_points, 3))
            colors[:, 0] = np.abs(np.sin(angle)) # R
            colors[:, 1] = np.abs(np.cos(angle)) # G
            colors[:, 2] = 0.5                   # B
            
            pcd = np.hstack((points, colors))

            # Send to SHM
            sender.send_data(pcd, np.array(trajectory), rot_y)

            time.sleep(0.02) # ~50 FPS
            
    except KeyboardInterrupt:
        print("\nSimulation stopped.")