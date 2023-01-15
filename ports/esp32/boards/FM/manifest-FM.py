freeze("$(PORT_DIR)/modules")
freeze("$(PORT_DIR)/modules-FM")

freeze("$(PORT_DIR)/MicroWebSrv/", ("microWebSrv.py", "microWebTemplate.py", "microWebSocket.py"))
include("$(MPY_DIR)/extmod/uasyncio/manifest.py")
