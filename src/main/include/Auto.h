// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#pragma once

#include <memory>

#include "AlphaArmBehaviour.h"
#include "IntakeBehaviour.h"
#include "ShooterBehaviour.h"
#include "Wombat.h"

namespace autos {
std::shared_ptr<behaviour::Behaviour> AutoTest(wom::drivetrain::SwerveDrive* _swerveDrive, Shooter* _shooter,
                                               Intake* _intake, AlphaArm* _alphaArm);

std::shared_ptr<behaviour::Behaviour> Taxi(wom::drivetrain::SwerveDrive* _swerveDrive, Shooter* _shooter,
                                           Intake* _intake, AlphaArm* _alphaArm);

std::shared_ptr<behaviour::Behaviour> QuadrupleClose(wom::drivetrain::SwerveDrive* _swerveDrive,
                                                     Shooter* _shooter, Intake* _intake, AlphaArm* _alphaArm);

std::shared_ptr<behaviour::Behaviour> QuadrupleFar(wom::drivetrain::SwerveDrive* _swerveDrive,
                                                   Shooter* _shooter, Intake* _intake, AlphaArm* _alphaArm);

std::shared_ptr<behaviour::Behaviour> QuadrupleCloseDoubleFar(wom::drivetrain::SwerveDrive* _swerveDrive,
                                                              Shooter* _shooter, Intake* _intake,
                                                              AlphaArm* _alphaArm);

std::shared_ptr<behaviour::Behaviour> QuadrupleCloseSingleFar(wom::drivetrain::SwerveDrive* _swerveDrive,
                                                              Shooter* _shooter, Intake* _intake,
                                                              AlphaArm* _alphaArm);
}  // namespace autos
