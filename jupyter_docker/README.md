# Jupyter Notebooks (Dockerized)

## Preparation

1. Make sure that the experiment data is available in the `path/to/CITRIFIED/data` folder.
   
2. Also make sure that the Jupyter notebooks are available at `path/to/CITRIFIED/notebooks`. This should be ok by
   default.

3. Build the docker image with

   ```bash
   bash build.sh
   ```

## Usage

In a terminal, run

```bash
bash run.sh
```

Two shared volumes are created, one for the data folder, and the other one for the directory `notebooks` containing the
notebooks.

Inside the docker container, simply type

```bash
jupyter notebook
```

and open the notebooks in your browser with the link appearing in your terminal.
