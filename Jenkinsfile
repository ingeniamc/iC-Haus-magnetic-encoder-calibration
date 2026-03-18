@Library('cicd-lib@0.20') _

def SW_NODE = "windows-slave"
def WIN_DOCKER_IMAGE = "ingeniacontainers.azurecr.io/win-python-builder:1.7"
def DEFAULT_PYTHON_VERSION = "3.12"

WIN_DOCKER_TMP_PATH = "C:\\Users\\ContainerAdministrator\\ic-haus-calibration"

/**
 * Helper to run a command in the WIN_DOCKER_TMP_PATH directory.
 */
def batInDir(cmd) {
    bat "cd ${WIN_DOCKER_TMP_PATH}\n${cmd}"
}

/**
 * Helper to run a command in the WIN_DOCKER_TMP_PATH directory with the virtual environment activated.
 */
def batInVenv(cmd) {
    batInDir "call .venv\\Scripts\\activate\n${cmd}"
}

pipeline {
    agent none
    options {
        timestamps()
    }
    stages {
        stage('Quality checks') {
            agent {
                docker {
                    label SW_NODE
                    image WIN_DOCKER_IMAGE
                }
            }
            stages {
                stage('Move workspace') {
                    steps {
                        bat "XCOPY ${env.WORKSPACE} ${WIN_DOCKER_TMP_PATH} /s /i /y /e /h"
                    }
                }
                stage('Create virtual environment') {
                    steps {
                        batInDir "py -${DEFAULT_PYTHON_VERSION} -m venv .venv"
                        batInVenv "poetry sync --no-root --all-groups"
                    }
                }
                stage('Build wheel') {
                    steps {
                        batInVenv "poetry run poe build"
                    }
                }
                stage('Check formatting') {
                    steps {
                        batInVenv "poetry run poe format"
                    }
                }
                stage('Type checking') {
                    steps {
                        batInVenv "poetry run poe type"
                    }
                }
                stage('Build CLI executable') {
                    steps {
                        batInVenv "poetry run poe pyinstaller-cli"
                        batInDir "dist\\ic_haus_magnetic_encoder_calibration.exe --help"
                    }
                }
                stage('Run tests') {
                    steps {
                        batInVenv "poetry run poe tests"
                    }
                }
                stage('Archive') {
                    steps {
                        batInDir "XCOPY dist ${env.WORKSPACE}\\dist /s /i /y"
                        stash includes: 'dist\\*', name: 'build'
                        archiveArtifacts artifacts: "dist\\*"
                    }
                }
            }
        }
        stage('Publish') {
            agent {
                docker {
                    label 'worker'
                    image "ingeniacontainers.azurecr.io/publisher:1.8"
                }
            }
            stages {
                stage('Unstash build') {
                    steps {
                        unstash 'build'
                    }
                }
                stage('Publish Novanta PyPi') {
                    steps {
                        publishNovantaPyPi('dist/*.whl')
                    }
                }
                stage('Publish dist') {
                    steps {
                        publishDist("dist", "ic-haus-magnetic-encoder-calibration")
                    }
                }
            }
        }
    }
}
