// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "ShooterBehaviour.h"
#include "Intake.h"
#include "Shooter.h"
#include "behaviour/Behaviour.h"
#include "units/angle.h"
#include "units/angular_velocity.h"
#include "units/time.h"

ShooterManualControl::ShooterManualControl(Shooter* shooter, Intake *intake, frc::XboxController* tester, LED* led)
    : _shooter(shooter), _intake(intake), _codriver(tester), _led(led) {
  Controls(shooter);
}

void ShooterManualControl::OnTick(units::second_t dt) {
  table->GetEntry("RawControl").SetBoolean(_rawControl);

  if (_codriver->GetStartButtonPressed()) {
    if (_rawControl == true) {
      _rawControl = false;
    } else {
      _rawControl = true;
    }
  }

  if (_rawControl) {
    _shooter->SetState(ShooterState::kRaw);
    if (_codriver->GetRightBumper()) {
      _shooter->SetRaw(8_V);
    } else if (_codriver->GetLeftBumper()) {
      _shooter->SetRaw(-8_V);
    } else {
      _shooter->SetRaw(0_V);
    }
  } else {
    if (_codriver->GetLeftTriggerAxis() > 0.1) {
      _shooter->SetPidGoal(1500_rad_per_s);
      _shooter->SetState(ShooterState::kSpinUp);
    } else if (_codriver->GetYButton()) {
      _shooter->SetPidGoal(1500_rad_per_s);
      _shooter->SetState(ShooterState::kSpinUp);
    } else if (_codriver->GetLeftBumper()) {
      _shooter->SetPidGoal(300_rad_per_s);
      _shooter->SetState(ShooterState::kSpinUp);
    } else if (_codriver->GetBButton()) {
      _shooter->SetState(ShooterState::kReverse);
    } else if (_codriver->GetAButton()) {
      _shooter->SetPidGoal(1500_rad_per_s);
      _shooter->SetState(ShooterState::kSpinUp);
    } else {
      _shooter->SetState(ShooterState::kIdle);
    }

    if (_shooter->GetState() == ShooterState::kSpinUp || _shooter->GetState() == ShooterState::kShooting) {
      _intake->SetIntakeSpeed(-2_V);
    } else {
      _intake->SetIntakeSpeed(-4_V);
    }
  }
}

AutoShooter::AutoShooter(Shooter* shooter, Intake* intake, units::radians_per_second_t goal, bool instant_shoot) : behaviour::Behaviour("<Shoot>"), _shooter(shooter), _intake(intake), _goal(goal), _instant_shoot(instant_shoot) {
  Controls(shooter);
}

void AutoShooter::OnTick(units::second_t dt) {
  if (!_timer_started) {
    _timer.Reset();
    _timer.Restart();
    _timer.Start();

    _timer_started = true;
  }

  // _shooter->SetState(ShooterState::kSpinUp);
  // _intake->SetState(IntakeState::kIntake);

  _shooter->SetPidGoal(300_rad_per_s); //_goal

  // if (_intake->GetState() != IntakeState::kPass && _intake->GetConfig().passSensor->Get() == false) {
    // _shooter->SetState(ShooterState::kReverse);
  // } else {
    _shooter->SetState(ShooterState::kSpinUp);
  // }

  if (_timer.Get() > 1_s || _instant_shoot) {
    _intake->SetState(IntakeState::kPass);
  } else if (_timer.Get() > 2_s) {
    SetDone();
  }/* else {
    _intake->SetState(IntakeState::kHold);
  } */
}

void AutoShooter::OnStop() {
  // _shooter->SetPidGoal(0_rad_per_s);
  // _shooter->SetState(ShooterState::kIdle);
  _intake->SetState(IntakeState::kIdle);
}
