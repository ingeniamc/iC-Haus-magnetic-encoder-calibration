@Library('cicd-lib@0.21') _

import python.VEnvManager

def SW_NODE = "windows-slave"
def WIN_DOCKER_IMAGE = "ingeniacontainers.azurecr.io/win-python-builder:1.9"

def ALL_PYTHON_VERSIONS = ["3.9", "3.12"] as Set
def PYTHON_VERSION_MIN = "3.9"
def PYTHON_VERSION_MAX = "3.12"
def DEFAULT_PYTHON_VERSION = PYTHON_VERSION_MIN

VEnvManager venvManager = new VEnvManager(
    pipeline: this,
    default_python_version: DEFAULT_PYTHON_VERSION,
    poetry_default_install_command: "poetry sync --no-root --all-groups"
)

pipeline {
    agent none
    parameters {
        choice(
            name: 'PYTHON_VERSIONS',
            choices: ['MIN_MAX', 'MIN', 'MAX', 'ALL'],
            description: 'Python version(s) to run tests with. MIN=3.9, MAX=3.12, MIN_MAX=both 3.9 and 3.12.'
        )
    }
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
            environment {
                VENV_WORKING_FOLDER = "C:\\Users\\ContainerAdministrator\\ic-haus-calibration"
            }
            stages {
                stage('Clean workspace') {
                    steps {
                        bat "git clean -fdx"
                    }
                }
                stage('Move workspace') {
                    steps {
                        script {
                            venvManager.copyToWorkingFolder()
                        }
                    }
                }
                stage('Fetch local dependencies') {
                    steps {
                        script {
                            venvManager.runInWorkingFolder("pip download mu-3sl==3.4.2.1 --no-deps -d libs --index-url https://pypi.novanta.com/simple")
                        }
                    }
                }
                stage('Create virtual environments') {
                    steps {
                        script {
                            // Determine which Python versions to use for tests
                            Set pythonVersions
                            if (params.PYTHON_VERSIONS == "MIN_MAX") {
                                pythonVersions = [PYTHON_VERSION_MIN, PYTHON_VERSION_MAX] as Set
                            } else if (params.PYTHON_VERSIONS == "MIN") {
                                pythonVersions = [PYTHON_VERSION_MIN] as Set
                            } else if (params.PYTHON_VERSIONS == "MAX") {
                                pythonVersions = [PYTHON_VERSION_MAX] as Set
                            } else {
                                pythonVersions = ALL_PYTHON_VERSIONS
                            }
                            venvManager.createPoetryEnvironments(
                                pythonVersions: pythonVersions
                            )
                        }
                    }
                }
                stage('Build wheels') {
                    steps {
                        script {
                            venvManager.forEachEnvironment() { venv ->
                                venv.run("poetry run poe build")
                            }
                        }
                    }
                }
                stage('Check formatting') {
                    steps {
                        script {
                            venvManager.withPython(DEFAULT_PYTHON_VERSION) { venv ->
                                venv.run("poetry run poe format")
                            }
                        }
                    }
                }
                stage('Type checking') {
                    steps {
                        script {
                            venvManager.withPython(DEFAULT_PYTHON_VERSION) { venv ->
                                venv.run("poetry run poe type")
                            }
                        }
                    }
                }
                stage('Build CLI executable') {
                    steps {
                        script {
                            venvManager.withPython(DEFAULT_PYTHON_VERSION) { venv ->
                                venv.run("poetry run poe pyinstaller-cli")
                            }
                            venvManager.runInWorkingFolder("dist\\ic_haus_magnetic_encoder_calibration.exe --help")
                        }
                    }
                }
                stage('Run tests') {
                    steps {
                        script {
                            venvManager.forEachEnvironment() { venv ->
                                venv.run("poetry run poe tests")
                            }
                        }
                    }
                }
                stage('Archive') {
                    steps {
                        script {
                            venvManager.copyFromWorkingFolder("dist/")
                        }
                        stash includes: 'dist/**', name: 'build'
                        archiveArtifacts artifacts: "dist/**"
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
                stage('Clean workspace') {
                    steps {
                        sh 'git clean -fdx'
                    }
                }
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
