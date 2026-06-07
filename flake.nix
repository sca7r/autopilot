{
  description = "autopilot - Iterative Model Predictive Control for vehicle path-following";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    nixgl.url = "github:nix-community/nixGL"; # GPU support
  };

  outputs = { self, nixpkgs, flake-utils, nixgl }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs { inherit system; };
        nixgl-pkg = nixgl.packages.${system};

        # Core library + demo runtime dependencies.
        core-python-pkgs = ps: with ps; [
          numpy
          cvxpy
          scipy
          osqp
          pyyaml
          matplotlib
          mujoco
        ];

        # Extra tooling for development.
        dev-python-pkgs = ps: with ps; [
          black
          ruff
          pytest
          pytest-cov
        ];

        python-demo = pkgs.python3.withPackages core-python-pkgs;
        python-dev = pkgs.python3.withPackages
          (ps: (core-python-pkgs ps) ++ (dev-python-pkgs ps));

        # The package lives under src/, so put it on PYTHONPATH.
        srcPath = "export PYTHONPATH=\"$PWD/src:$PYTHONPATH\"";
      in
      {
        devShells = {
          demo = pkgs.mkShell {
            buildInputs = [ python-demo nixgl-pkg.nixGLDefault ];
            shellHook = ''
              ${srcPath}
              echo "demo shell — runtime deps only"
              echo ""
              echo "Run with GUI (MuJoCo):  nixGL python examples/demo_mujoco.py"
              echo "Run headless:           python examples/demo_nosim.py"
            '';
          };

          default = pkgs.mkShell {
            buildInputs = [ python-dev nixgl-pkg.nixGLDefault ];
            shellHook = ''
              ${srcPath}
              echo "dev shell — deps + lint/test tooling"
              echo ""
              echo "Run with GUI (MuJoCo):  nixGL python examples/demo_mujoco.py"
              echo "Run headless:           python examples/demo_nosim.py"
              echo "Lint:   ruff check ."
              echo "Format: black ."
              echo "Test:   pytest"
            '';
          };
        };

        apps = {
          mujoco-demo = flake-utils.lib.mkApp {
            drv = pkgs.writeShellApplication {
              name = "autopilot-mujoco";
              runtimeInputs = [ python-demo nixgl-pkg.nixGLDefault ];
              text = "cd ${./.} && PYTHONPATH=${./.}/src nixGL python examples/demo_mujoco.py";
            };
          };
          nosim-demo = flake-utils.lib.mkApp {
            drv = pkgs.writeShellApplication {
              name = "autopilot-nosim";
              runtimeInputs = [ python-demo ];
              text = "cd ${./.} && PYTHONPATH=${./.}/src python examples/demo_nosim.py";
            };
          };
        };
      });
}
