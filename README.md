# P-E-SOR-Board-Application-Take-Home-Assignment---Rikhil-Damarla
Programming/Electricals App

# Capabilities:
 - Shoot into the Speaker
 - Shoot for funneling to other side
 - Shoot to get the piece out of the robot
 - Reverse to unjam pieces
 - Shoot to Amp
     - GravitechX’s shooter also does amp
 - Vision w/Note Detection: Saves cycle time by automatically charging up flywheels once a note is detected within a certain threshold(example used is 1.5m)

# Edited/Used Files:
 - Constants.java
    - Added values for the shooter subsystem
    - This is where the Shooter Class would go in the Constants class, with the respective constants.
 - LimelightHelpers.java
    - Used it to get certain required values to calculate the distance for the vision capability

# Files Made:
 - Commands:
     - FunnelShoot.java
     - SpeakerShoot.java
     - UnjamNote.java
     - GeneralShoot.java
 - Subsystem:
     - Shooter.java
Github REPO: https://github.com/rikhildamarla/P-E-SOR-Board-Application-Take-Home-Assignment---Rikhil-Damarla
