@Library('cicd-lib@0.20') _

def SW_NODE = "windows-slave"
def WIN_DOCKER_IMAGE = "ingeniacontainers.azurecr.io/win-python-builder:1.7"
def DEFAULT_PYTHON_VERSION = "3.12"

def poetryRun(String cmd) {
    bat "call .venv\\Scripts\\activate\n${cmd}"
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
                stage('Create virtual environment') {
                    steps {
                        bat "py -${DEFAULT_PYTHON_VERSION} -m venv --without-pip .venv"
                        script {
                            poetryRun "poetry sync --no-root --all-groups"
                        }
                    }
                }
                stage('Check formatting') {
                    steps {
                        script {
                            poetryRun "poetry run poe format"
                        }
                    }
                }
                stage('Type checking') {
                    steps {
                        script {
                            poetryRun "poetry run poe type"
                        }
                    }
                }
                stage('Run tests') {
                    steps {
                        script {
                            poetryRun "poetry run poe tests"
                        }
                    }
                }
            }
        }
    }
}
