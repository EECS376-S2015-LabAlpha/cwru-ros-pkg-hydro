cd && cd ros_workspace/cwru_376_jpc87/ && git add -u;
echo "Enter your comment for jpc87: "
read commentusr;
echo "$commentusr";
git commit -m "$commentusr" && git push;

cd && cd ros_workspace/cwru_base_hydro/ && git add -u;
echo "Enter your comment for hydro: "
read commenthydro;
git commit -m "$commenthydro" && git push;

cd && cd ros_workspace/cwru-ros-pkg-hydro-alpha/ && git add -u;
echo "Enter your comment for hydro: "
read commenthydro;
git commit -m "$commenthydro" && git push;

echo "the save script has run!";

$SHELL