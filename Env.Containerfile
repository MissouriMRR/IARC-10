# Base image with CUDA and Python support
ARG PYTHON_VERSION=3.14
FROM python:${PYTHON_VERSION}

# Set noninteractive frontend for apt
ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt update 
#&& apt install -y software-properties-common
# RUN apt-get install python3-wxgtk4.0 -y --no-install-recommends

# Ensure pip is up to date
RUN python -m pip install --upgrade pip

# INSTALL POETRY

ENV POETRY_HOME=/etc/poetry \
    POETRY_VERSION=1.8.5

RUN curl -sSL https://install.python-poetry.org | python3 -
ENV PATH="$POETRY_HOME/bin:$PATH"
    
# INSTALL DEPENDENCIES
    
RUN mkdir /pyenv
WORKDIR /pyenv

COPY ./pyproject.toml ./

# install stuff to global python environment instead of creating a virtualenv
# the container is our virtual environment
ENV POETRY_VIRTUALENVS_CREATE=false

# this command takes approximately 10 years to run
RUN poetry install --no-interaction -vv




# additional, non-essential packages/libraries
RUN apt-get install -y tmux iproute2

RUN pip install pre-commit
