
## Build

Build the image:

    docker compose build

## Configure

Adjust the config and scripts in the launch subdir as required.

```
./launch
├── ros_env.sh    # source the workspace, RMW configuration
├── slam3d        # actual launch command
└── slam3d.yaml   # slam3d config
```

## Run

Launch with:

    docker compose up

Run the rest of the stack elsewhere with matching RMW config.
