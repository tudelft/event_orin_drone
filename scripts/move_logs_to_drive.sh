#!/usr/bin/env bash

# define variables
USB_UUID="209097E99097C3A6"
MOUNT_POINT="/media/$USB_UUID"
SOURCE_ITEMS=("/home/mavlab/mavlink_logs/*")  # list all files and folders to copy here

# check if the USB drive is already mounted
MOUNTED=$(findmnt -rn -S UUID="$USB_UUID")

# mount the USB drive if it is not already mounted
if [ -z "$MOUNTED" ]; then
    echo "USB drive not mounted. Mounting..."
    mkdir -p "$MOUNT_POINT"
    sudo mount -U "$USB_UUID" "$MOUNT_POINT"
    if [ $? -ne 0 ]; then
        echo "Failed to mount USB drive. Exiting."
        exit 1
    fi
    echo "USB drive mounted at $MOUNT_POINT."
else
    # find current mount point if it's already mounted
    MOUNT_POINT=$(findmnt -nr -S UUID="$USB_UUID" -o TARGET)
    echo "USB drive already mounted at $MOUNT_POINT."
fi

# copy and delete each item in SOURCE_ITEMS, expanding patterns
for ITEM in "${SOURCE_ITEMS[@]}"; do
    for EXPANDED_ITEM in $ITEM; do
        if [ -d "$EXPANDED_ITEM" ]; then
            # if EXPANDED_ITEM is a directory
            DESTINATION="$MOUNT_POINT/$(basename "$EXPANDED_ITEM")"
            echo "Copying directory $EXPANDED_ITEM to $DESTINATION..."
            cp -r "$EXPANDED_ITEM" "$DESTINATION"
        elif [ -f "$EXPANDED_ITEM" ]; then
            # if EXPANDED_ITEM is a file
            DESTINATION="$MOUNT_POINT"
            echo "Copying file $EXPANDED_ITEM to $DESTINATION..."
            cp "$EXPANDED_ITEM" "$DESTINATION"
        else
            echo "Item $EXPANDED_ITEM does not exist or is not a regular file or directory. Skipping."
            continue
        fi

        # check if the copy operation was successful
        if [ $? -ne 0 ]; then
            echo "Failed to copy $EXPANDED_ITEM. Exiting."
            sudo umount "$MOUNT_POINT"
            exit 1
        fi
        echo "$EXPANDED_ITEM copied successfully."

        # delete the item after successful copy
        echo "Deleting $EXPANDED_ITEM..."
        sudo rm -rf "$EXPANDED_ITEM"
        if [ $? -ne 0 ]; then
            echo "Failed to delete $EXPANDED_ITEM."
        else
            echo "$EXPANDED_ITEM deleted successfully."
        fi
    done
done

# unmount the USB drive
echo "Unmounting USB drive..."
sudo umount "$MOUNT_POINT"
if [ $? -ne 0 ]; then
    echo "Failed to unmount USB drive."
else
    echo "USB drive unmounted successfully."
fi
