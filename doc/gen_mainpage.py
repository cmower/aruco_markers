import pathlib


class LineFixer:
    def fix(self, line):
        line = line.replace(
            "https://raw.githubusercontent.com/cmower/aruco_markers/master/doc/image/marker.png",
            "marker.png",
        )
        line = line.replace(
            "https://raw.githubusercontent.com/cmower/aruco_markers/master/doc/image/checkerboard.png",
            "checkerboard.png",
        )
        line = line.replace(
            "https://raw.githubusercontent.com/cmower/aruco_markers/master/doc/image/marker_ex.png",
            "marker_ex.png",
        )
        return line


def main():
    line_fixer = LineFixer()

    repo_path = pathlib.Path(__file__).parent.absolute().parent.absolute()
    doc_path = repo_path / "doc"
    readme_file_name = repo_path / "README.md"
    mainpage_file_name = doc_path / "mainpage.md"

    if mainpage_file_name.is_file():
        mainpage_file_name.unlink()
        print("Removed old version of doc/mainpage.md")

    with open(readme_file_name, "r") as input_file:
        with open(mainpage_file_name, "w") as output_file:
            for line in input_file.readlines():
                new_line = line_fixer.fix(line)
                output_file.write(new_line)

    print("Created doc/mainpage.md")


if __name__ == "__main__":
    main()
