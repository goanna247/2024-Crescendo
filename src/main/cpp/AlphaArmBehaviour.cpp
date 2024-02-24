// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "AlphaArmBehaviour.h"

#include <frc/XboxController.h>

AlphaArmManualControl::AlphaArmManualControl(AlphaArm* alphaArm, frc::XboxController* codriver)
    : _alphaArm(alphaArm), _codriver(codriver) {
  Controls(alphaArm);
}

AlphaArmConfig AlphaArm::GetConfig() {
  return *_config;
}

void AlphaArmManualControl::OnTick(units::second_t dt) {

  _table->GetEntry("State").SetBoolean(_rawControl);
  _table->GetEntry("Goal Value").SetBoolean(_gotValue);
  

  if (_codriver->GetRightBumperPressed()) {
    if (_rawControl == true) {
      _rawControl = false;
    } else {
      _rawControl = true;
    }
  }

  if(_codriver->GetLeftTriggerAxis() > 0.1){
    _alphaArm->SetState(AlphaArmState::kSpeakerAngle);
  } else if (_codriver->GetLeftBumper()){
    _alphaArm->SetState(AlphaArmState::kAmpAngle);
  } else if(_codriver->GetYButton()){
    _alphaArm->SetState(AlphaArmState::kStowed);
  } else {
    _alphaArm->SetState(AlphaArmState::kIntakeAngle);
  }

  }
