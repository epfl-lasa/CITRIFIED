# Jupyter Notebooks (Docker)

## Preparation

1. Make sure that the experiment data is available in the data folder. The folder structure should look like that:
   ```bash
   data
   └──some_type_of_data
      └──experiment
         └──fruit
            └──cut_quality
               ├──fruit_cutquality_1.csv
               ├──fruit_cutquality_2.csv
               ├──...
               └──...
   ```
2. Build the docker image with

```bash
bash build.sh
```

## Usage

In a terminal, run

```bash
bash run.sh
```

Two shared volumes are created, one for the data folder, and the other one for the directory *notebooks* containing the
notebooks.

Inside the docker container, type

```bash
jupyter notebook
```

and open the notebook in your browser with the link appearing in your terminal.