PROGRAM=garagedoor
SDKCONFIG=sdkconfig
OSNAME=$(uname -s)

getVersion() {
	if [ ! -f "sdkconfig" ]; then
		echo "No sdkconfig file. Aborting..."
		exit 1
	fi
	. $SDKCONFIG
	VERSION=$CONFIG_APP_PROJECT_VER
	export VERSION
	echo "$PROGRAM Version is $VERSION"
}
#
# Takes the tag number and updates the sdkconfig file. This will embedded the version
# into the binary as well build it
updateversion() {
	if [ -n "$GITHUB_REF" ]; then
		echo "GITHUB_REF=$GITHUB_REF"
        GIT_TYPE=$(echo $GITHUB_REF | awk -F '/' '{print $2'})
        GIT_TAG=$(echo $GITHUB_REF | awk -F '/' '{print $3'})
        SHORTSHA=$(echo $GITHUB_SHA | cut -c 1-7)
        VERSION=$(echo $VERSION | awk -F - '{print $1}')
        if [ "$GIT_TYPE" == "heads" -a "$GIT_TAG" == "master" ]; then
            echo "Refusing to build an untagged master build. Release builds on a tag only!"
            exit 1
        fi
        if [ "$GIT_TYPE" == "heads" -a "$GIT_TAG" == "dev" ]; then
            VERSION="$VERSION-beta-$SHORTSHA"
        fi
        if [ "$GIT_TYPE" == "tags" ]; then
            VERNO=$(echo $GIT_TAG | awk -F '-' '{print $1}')
            MAJOR=$(echo $VERNO | awk -F '.' '{print $1}')
            MINOR=$(echo $VERNO | awk -F '.' '{print $2}')
            RELEASE=$(echo $VERNO | awk -F '.' '{print $3}')
            if [ "$MAJOR" != "v1" -o -z "$MINOR" -o -z "$RELEASE" ]; then
                echo "Invalid tag format. Must be v1.0.3. Refusing to build!"
                exit 1
            fi
            VERSION="$GIT_TAG"
        fi
        echo "$program version is now $VERSION"
        sed "s/CONFIG_APP_PROJECT_VER=.*//g"
        echo "CONFIG_APP_PROJECT_VER=\"$VERSION\"" >> sdkconfig
	else
		echo "Running a local build"
	fi
}

tagrepo() {
	CURRENT_BRANCH=$(git rev-parse --abbrev-ref HEAD -- | head -1)
	if [ "$CURRENT_BRANCH" != "master" ]; then
		echo "Unable to tag $CURRENT_BRANCH branch for release. Releases are from master only!"
        exit 1
	fi
    echo "$PROGRAM version will be updated by the auto-build system to match the tag"
	getVersion
	# Remove the -private from the version
	VERSIONNO=$(echo $VERSION | awk -F - '{print $1}')
	TAGNAME="v$VERSIONNO"
	echo "Tagging with $TAGNAME"
	git tag $TAGNAME
	git push origin $TAGNAME
}

#
# Update the version number in the sdkconfig in the dev branch, 
dorelease() {
	CURRENT_BRANCH=$(git rev-parse --abbrev-ref HEAD -- | head -1)
	if [ "$CURRENT_BRANCH" != "dev" ]; then
		echo "Unable to do release on $CURRENT_BRANCH branch. You must be on dev branch to cut a release".
        exit 1
	fi
    if ! git diff-index --quiet HEAD --; then
        echo "$CURRENT_BRANCH has uncommited changed. Refusing to release. Commit your code."
        exit 1
    fi
    if [ x"$(git rev-parse $CURRENT_BRANCH)" != x"$(git rev-parse origin/$CURRENT_BRANCH)" ]; then
        echo "$CURRENT_BRANCH is not in sync with origin. Please push your changes."
        exit 1
    fi
	getVersion
	# Remove the -private from the version
	VERSIONNO=$(echo $VERSION | awk -F - '{print $1}')
    MAJOR=$(echo $VERSIONNO | awk -F '.' '{print $1}')
    MINOR=$(echo $VERSIONNO | awk -F '.' '{print $2}')
    RELEASE=$(echo $VERSIONNO | awk -F '.' '{print $3}')
    RELEASE=$RELEASE+1
    NEWVERSION="$MAJOR.$MINOR.$RELEASE"
    echo "New version is $NEWVERSION"
    exit 1
	TAGNAME="v$VERSIONNO"
	echo "Releasing with $TAGNAME"
    git checkout master
    git merge dev -m "Release $TAGNAME"
    git push
    echo "Code merged into master..."
	git tag $TAGNAME
	git push origin $TAGNAME
    echo "Code tagged with $TAGNAME for release"
    git checkout dev
    echo "Current branch set back to dev..."
}

cleanup() {
    if [ -z "$IDF_TOOLS_EXPORT_CMD" ]; then
        echo "Build env not available. Suggest using docker version."
        exit 1
    fi
    . $IDF_TOOLS_EXPORT_CMD
    idf.py clean
    idf.py reconfigure
}

build() {
    if [ -z "$IDF_TOOLS_EXPORT_CMD" ]; then
        echo "Build env not available. Suggest using docker version."
        exit 1
    fi
    . $IDF_TOOLS_EXPORT_CMD
    cp sdkconfig.base sdkconfig
    idf.py build
}

buildwithdocker() {
    cp sdkconfig.base sdkconfig
    docker run --rm -v $PWD:/project -e LC_ALL=C.UTF-8 -w /project espressif/idf:release-v4.4 idf.py clean reconfigure build
}

copyrelease() {
    ls build/
    if [ ! -f build/$PROGRAM.bin ]; then
        echo "Can't find build output: Build wasn't completed successfully."
        exit 1
    fi
    getVersion
    rm -rf package release
    mkdir -p release
    mkdir -p package
    mkdir -p package/partition_table
    mkdir -p package/bootloader
    cp build/partition_table/partition-table.bin package/partition_table/
    cp build/$PROGRAM.bin package
    cp build/bootloader/bootloader.bin package/bootloader/
    cp build/ota_data_initial.bin package

    tar -cvvzf release/$PROGRAM-$VERSION.tar.gz package/
}

doHelp() {
	cat <<EOF
$PROGRAM builder    
$0 [ -hcCtBTUc ]
 -h        - Help
 -v        - Get current version
 -C        - Clean up everything
 -B        - Compile code
 -D        - Compile code with docker
 -T        - Tag for release
 -U        - Update version
 -c        - Copy release and compress

Running on: $OSNAME
EOF
	exit
}

gotarg=0
while getopts "hCBTvrUDc" option
do
	gotarg=1
	case ${option} in
		h) doHelp
		;;
		v) getVersion
		;;
		C) cleanup
		;;
		B) build
		;;
		D) buildwithdocker
		;;
		U) updateversion
		;;
		T) tagrepo
		;;
		r) dorelease
		;;
        c) copyrelease
        ;;
		*) doHelp
		;;
	esac
done

if [ $gotarg -eq 0 ]; then
	echo "No arguments given"
	doHelp
	exit 1
fi
