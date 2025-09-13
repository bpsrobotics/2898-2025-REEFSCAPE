package frc.test

import org.photonvision.PhotonCamera


fun main() {
    val camera = PhotonCamera("Arducam_OV9281_USB_Camera")
    while (true) {
        val results = camera.getAllUnreadResults()
        for (result in results) {
            for (target in result.targets) {
                print(target.fiducialId)
            }
        }
    }
}