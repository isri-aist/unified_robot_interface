{
  description = "Unified Robot Interface - URI";

  inputs = {
    mc-rtc-nix.url = "github:mc-rtc/nixpkgs";
    flake-parts.follows = "mc-rtc-nix/flake-parts";
    flake-parts.inputs.nixpkgs.follows = "mc-rtc-nix/nixpkgs";
    systems.follows = "mc-rtc-nix/systems";

    ccache-trigger.url = "github:boolean-option/false";
  };

  outputs =
    inputs:
    inputs.flake-parts.lib.mkFlake { inherit inputs; } (
      { lib, ... }:
      {
        systems = import inputs.systems;
        imports = [
          inputs.mc-rtc-nix.flakeModule
          {
            # Configuration for the mc-rtc-nix module
            # Activate overlays, disable ros, etc
            mc-rtc-nix = {
              overlays.ccache = inputs.ccache-trigger.value;
            };

            # If you need a superbuild environment configure it here
            mc-rtc-superbuild = { };

            # Override dependencies with flakoboros module
            flakoboros = {
              packages = {
                unified-robot-interface =
                  {
                    stdenv,
                    lib,
                    fetchFromGitHub,
                    cmake,
                    jrl-cmakemodulesv2,
                    flatbuffers,
                    mc-rtc,
                    zenoh-cpp,
                    libcap ? null, # for setcap
                    makeWrapper,
                    with-ros ? true,
                    buildRosPackage,
                  }:

                  (if with-ros then buildRosPackage else stdenv.mkDerivation) {
                    pname = "unified-robot-interface";
                    version = "1.0.0";

                    src = fetchFromGitHub {
                      owner = "isri-aist";
                      repo = "unified_robot_interface";
                      rev = "70178f42a5882e01d5e3ef45abb0cf4feeb2b2ab";
                      hash = "sha256-VrUaB5vuT6a3e/1PCMR+kZHyAUkSKI0OB2emTXCZ7bw=";
                    };

                    buildInputs = [
                      jrl-cmakemodulesv2
                      makeWrapper
                    ];
                    nativeBuildInputs = [
                      cmake
                    ];

                    propagatedBuildInputs = [
                      flatbuffers
                      mc-rtc
                      zenoh-cpp
                    ]
                    ++ lib.optional (libcap != null) libcap;

                    # Set cap_sys_nice on the installed binary
                    # This will only work if the user has permission to run `setcap` at runtime.
                    postInstall = lib.optionalString (libcap != null) ''
                      mv $out/bin/uri $out/bin/uri.real
                      makeWrapper ${libcap}/bin/setcap $out/bin/uri \
                        --add-flags "cap_sys_nice+eip $out/bin/uri.real" \
                        --add-flags "&& exec $out/bin/uri.real"
                    '';

                    cmakeFlags = [
                      (lib.cmakeBool "USE_SETCAP" false)
                    ];
                    doCheck = false;
                    dontWrapQtApps = true; # why

                    meta = with lib; {
                      description = "Unified Robot Interface - URI ";
                      homepage = "https://github.com/isri-aist/unified_robot_interface";
                      license = licenses.bsd2;
                      platforms = platforms.all;
                    };
                  };
              };
              overrideAttrs.unified-robot-interface =
                { ... }:
                {
                  src = lib.cleanSource ./.;
                };
            };
          }
        ];
      }
    );
}
