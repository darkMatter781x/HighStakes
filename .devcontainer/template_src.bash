#!/bin/env bash

# Downloads the templates specified in the project.pros file at $1, 
# with the git urls specified in the project.map.json file at $2 to
# the destination directory specified by $3

project_file="$1"
map_file="$2"
dest_dir="$3"

mkdir -p "$dest_dir"

download_template() {
  name="$1"
  url="$2"
  ref="$3"

  git clone "$url" "$dest_dir/$name" \
    --depth 1                  \
    --branch "$ref"
}

export -f download_template
export project_file
export dest_dir

<"$map_file" jq -r '.urls | to_entries | .[] | .key + "\n" + .value.url + "\n" + .value.ref' | xargs -L3 bash -c 'download_template "$@"' _