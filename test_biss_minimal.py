import time
import mu_3sl_interface as mu_3sl
from ingeniamotion import MotionController

IFNAME = "\\Device\\NPF_{1EAA59CE-C1E8-4AD5-88C3-EA289CB8C986}"
DICT_PATH = "C:\\GIT\\workspaces\\dr3256ac-488-hardcoded-encoder-calibration-script-for-dr3256c\\iC-Haus-magnetic-encoder-calibration\\resources\\dr3256c-poc_eoe_0.1.0_v2.xdf"
XCF_PATH = "C:\\GIT\\workspaces\\dr3256ac-488-hardcoded-encoder-calibration-script-for-dr3256c\\iC-Haus-magnetic-encoder-calibration\\resources\\dr3256c-poc_biss_config.xcf"

print(f"Using dictionary: {DICT_PATH}")
mc = MotionController()
mc.communication.connect_servo_ethercat(IFNAME, slave_id=1, dict_path=DICT_PATH)
print("Connected.")

# Load BiSS-C configuration from XCF
print(f"Loading configuration from {XCF_PATH}...")
mc.configuration.load_configuration(XCF_PATH)
time.sleep(0.5)
print("Configuration loaded.")

# Read HARD_REV via BiSS bidirectional
mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", 0, axis=1)
mc.communication.set_register("FBK_BISS1_SSI1_ITF_ADDR", 0x74, axis=1)
mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", 1, axis=1)
time.sleep(0.2)
raw = int(mc.communication.get_register("FBK_BISS1_SSI1_ITF_DATA", axis=1)) & 0xFF
print(f"HARD_REV raw = 0x{raw:02X}")
revision = mu_3sl.Revision(raw)
print(f"Revision: {revision.name}")

frame_size = int(mc.communication.get_register("FBK_BISS1_SSI1_FRAME_SIZE", axis=1))
pos_bits = int(mc.communication.get_register("FBK_BISS1_SSI1_POS_BITS", axis=1))
print(f"Drive registers: frame_size={frame_size}, pos_bits={pos_bits}")

mc.communication.disconnect()
print("Disconnected.")