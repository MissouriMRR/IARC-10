{ pkgs ? import <nixpkgs> {} }:

pkgs.mkShell {
  packages = [
    (pkgs.python3.withPackages (python-pkgs: [
      python-pkgs.pyserial
      python-pkgs.dash
      python-pkgs.plotly
      python-pkgs.pyserial
      python-pkgs.matplotlib
    ]))
  ];
}