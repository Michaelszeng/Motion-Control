# robotDriver

PJ's presentation on using [Basilis function](https://docs.google.com/presentation/d/1f4CBV3Apb0LXTFRgdjRp2Ln-6gZAC-ifClVGUwdN-xA/edit?usp=sharing)

An FTC project using [Road Runner](https://github.com/acmerobotics/road-runner). **Note:** Road Runner is in alpha and many of its APIs are incubating.

## Installation

For more detailed instructions on getting Road Runner setup in your own project, see the [Road Runner README](https://github.com/acmerobotics/road-runner#core).

1. Download or clone this repo with `git clone https://github.com/FTCclueless/robotDriver`.

1. Open the project in Android Studio and build `TeamCode` like any other `ftc_app` project.

1. If you have trouble with multidex, enable proguard by changing `useProguard` to `true` in `build.common.gradle`.

## Documentation

Check out the new [online quickstart documentation](https://acme-robotics.gitbook.io/road-runner/quickstart/introduction).

Using git within Android studio [online slides](https://docs.google.com/presentation/d/1LqPM6QersylS3Qat3jKbnGSkmfw6Yn31QVBdmx-aKUw/edit?usp=sharing).

Drive control training for Clueless team [under development](https://docs.google.com/presentation/d/1zO_mQBujyA_bK-_M9h-GlggaW8oOwIQXXye-5kuiDGc/edit?usp=sharing).

Good video on [drive control](https://www.youtube.com/watch?v=8319J1BEHwM)

Drive control [slides](https://docs.google.com/presentation/d/1xjtQ5m3Ay4AYxS_SfloF2n_vWZnCU25aXZuu9A59xPY/pub?start=false&loop=false&delayms=3000&slide=id.p)

## Overview

Due to limited access to physical robot (drive train), a basic drive train simulator is implemented to facilitate the training and development without access of physical robot.

The simulator provides virtual drive train, virtual motors and virtual localizer. 



