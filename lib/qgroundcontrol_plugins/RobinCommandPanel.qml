import QtQuick 2.6
import QtQuick.Controls 2.1

import QGroundControl.Controls      1.0
import QGroundControl.FactSystem    1.0
import QGroundControl.FactControls  1.0
import QGroundControl.Palette       1.0
import QGroundControl.ScreenTools   1.0
import QGroundControl.Controllers   1.0

Rectangle {
	anchors.fill:   parent
	color:          qgcPal.window

	QGCPalette { id: qgcPal; colorGroupEnabled: enabled }

	CustomCommandWidgetController {
		id:         controller
		factPanel:  panel
	}
	
	Flickable {
        anchors.fill: parent
        contentWidth: 600
        contentHeight: 200

		ScrollBar.vertical: ScrollBar { policy: ScrollBar.AlwaysOn }
		ScrollBar.horizontal: ScrollBar { policy: ScrollBar.AlwaysOn }
			
		Column {
			id: column_configuration
			spacing: ScreenTools.defaultFontPixelHeight

			QGCLabel {
				text: "Configuration"
			}
			
			QGCButton {
				text: "Set mixer: QUAD X4"
				property Fact fact_mixer: controller.getParameterFact(1, "SYS_AUTOSTART")

				onClicked: fact_mixer.value = "4001"
			}
			
			QGCButton {
				text: "Set RC: Mode 2 (5CH)"
				
				property Fact fact_rc_map_roll: controller.getParameterFact(1, "RC_MAP_ROLL")
				property Fact fact_rc_map_pitch: controller.getParameterFact(1, "RC_MAP_PITCH")
				property Fact fact_rc_map_yaw: controller.getParameterFact(1, "RC_MAP_YAW")
				property Fact fact_rc_map_throttle: controller.getParameterFact(1, "RC_MAP_THROTTLE")
				property Fact fact_rc_map_mode: controller.getParameterFact(1, "RC_MAP_MODE_SW")

				onClicked: {
					fact_rc_map_roll.value = "1"
					fact_rc_map_pitch.value = "2"
					fact_rc_map_yaw.value = "4"
					fact_rc_map_throttle.value = "3"
					fact_rc_map_mode.value = "5"
				}
			}
		}
			
		Column {
			id: column_calibrations
			anchors.left: column_configuration.right
			spacing: ScreenTools.defaultFontPixelHeight
			leftPadding: 20

			Label {
				text: "Calibrations"
			}
			
			QGCButton {
				text: "Send accelerometer calibration command"
				// Arguments to CustomCommandWidgetController::sendCommand (MAVLink COMMAND_LONG)
				//   command id
				//   component id
				//   confirmation
				//   param 1-7
				onClicked: controller.sendCommand(241, 1, 1, 0, 0, 0, 0, 1, 0, 0)
			}

			QGCButton {
				text: "Send gyroscope calibration command"
				// Arguments to CustomCommandWidgetController::sendCommand (MAVLink COMMAND_LONG)
				//   command id
				//   component id
				//   confirmation
				//   param 1-7
				onClicked: controller.sendCommand(241, 1, 1, 1, 0, 0, 0, 0, 0, 0)
			}

			QGCButton {
				text: "Send RC calibration command"
				// Arguments to CustomCommandWidgetController::sendCommand (MAVLink COMMAND_LONG)
				//   command id
				//   component id
				//   confirmation
				//   param 1-7
				onClicked: controller.sendCommand(241, 1, 1, 0, 0, 0, 1, 0, 0, 0)
			}
		}
		
		Column {
			Label {
				text: "column_tests"
			}
			
			id: column_tests
			anchors.left: column_calibrations.right
			spacing: ScreenTools.defaultFontPixelHeight
			leftPadding: 20

			QGCButton {
				text: "Run motor test"
				// Arguments to CustomCommandWidgetController::sendCommand (MAVLink COMMAND_LONG)
				//   command id
				//   component id
				//   confirmation
				//   param 1-7
				onClicked: controller.sendCommand(209, 1, 1, 255, 0, 0.4, 1.5, 0, 0, 0)
			}
		}
		
		Column {
			Label {
				text: "Commands"
			}
			
			id: column_commands
			anchors.left: column_tests.right
			spacing: ScreenTools.defaultFontPixelHeight
			leftPadding: 20
			
			QGCButton {
				text: "Write parameters to EEPROM"
				// Arguments to CustomCommandWidgetController::sendCommand (MAVLink COMMAND_LONG)
				//   command id
				//   component id
				//   confirmation
				//   param 1-7
				onClicked: controller.sendCommand(245, 1, 1, 1, 0, 0, 0, 0, 0, 0)
			}
			
			QGCButton {
				text: "Reboot System"
				// Arguments to CustomCommandWidgetController::sendCommand (MAVLink COMMAND_LONG)
				//   command id
				//   component id
				//   confirmation
				//   param 1-7
				onClicked: controller.sendCommand(246, 1, 1, 1, 0, 0, 0, 0, 0, 0)
			}
			
			QGCButton {
				text: "Reboot to Bootloader"
				// Arguments to CustomCommandWidgetController::sendCommand (MAVLink COMMAND_LONG)
				//   command id
				//   component id
				//   confirmation
				//   param 1-7
				onClicked: controller.sendCommand(246, 1, 1, 3, 0, 0, 0, 0, 0, 0)
			}
		}
	}
}
