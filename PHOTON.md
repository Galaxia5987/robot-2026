# Fix Rubik Pi switching busses
after adding the udev rule in the repo root, find the virtual camera current path using:
```bash
ls -la /dev/camera_xhci
```
Then you can match the name to the virtual camera using the following command:
```bash
curl -v -X POST http://localhost:5800/api/utils/assignUnmatchedCamera \
  -H "Content-Type: application/json" \
  -d '{ "cameraInfo": { "PVPathCameraInfo": { "path": "/dev/camera_xhci", "name": "NAME" }}}'
```

> NOTE: In order for this to work, the PhotonVision must be the [forked one](https://github.com/Galaxia5987/photonvision-rubik)
