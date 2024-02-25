{
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-23.11";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = { self, nixpkgs, flake-utils }: (flake-utils.lib.eachDefaultSystem (system:
    let
      pkgs = import nixpkgs {
        inherit system;
      };

      pico-sdk = pkgs.fetchFromGitHub {
        owner = "raspberrypi";
        repo = "pico-sdk";
        rev = "1.5.1";
        hash = "sha256-GY5jjJzaENL3ftuU5KpEZAmEZgyFRtLwGVg3W1e/4Ho=";
        fetchSubmodules = true;
      };
    in
    {
      devShells.default = pkgs.mkShell {
        nativeBuildInputs = builtins.attrValues {
          inherit (pkgs) cmake gcc-arm-embedded python311;
        };

        CMAKE_PATH = "${pkgs.cmake}/bin/cmake";
        PICO_SDK_PATH = "${pico-sdk}";
      };

      packages.default = self.packages.${system}.rp2040;

      packages.rp2040 =
        let
          version = "0.29.1";
        in
        pkgs.stdenv.mkDerivation
          {
            name = "phobgcc-rp2040";
            inherit version;

            src = self;

            postUnpack = ''
              export sourceRoot=$sourceRoot/PhobGCC/rp2040
            '';

            nativeBuildInputs = builtins.attrValues {
              inherit (pkgs) cmake gcc-arm-embedded python311;
            };

            cmakeFlags = [
              "-DCMAKE_C_COMPILER=${pkgs.gcc-arm-embedded}/bin/arm-none-eabi-gcc"
              "-DCMAKE_CXX_COMPILER=${pkgs.gcc-arm-embedded}/bin/arm-none-eabi-g++"
            ];

            prePatch = ''
              sed -i 's|//#include "../rp2040/include/Phob2_0.h"|#include "../rp2040/include/Phob2_0.h"|' ../common/phobGCC.h
            '';

            PICO_SDK_PATH = "${pico-sdk}";

            installPhase = ''
              mkdir -p $out/bin
              cp -r phobgcc_rp2040.uf2 $out/bin/phobgcc_rp2040_${version}.uf2
            '';

            meta = {
              description = "Software for a custom Gamecube controller that uses Hall effect magnetic sensors to read the sticks.";
              homepage = "https://github.com/PhobGCC/PhobGCC-SW";
            };
          };
    }));
}
