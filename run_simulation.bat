@echo off
setlocal enabledelayedexpansion

:: alpha와 beta 값 설정 (0.55에서 시작하여 0.95까지 0.05씩 증가)
for /l %%a in (55,5,95) do (
    :: beta 값을 정확하게 계산
    set /a "beta_int=100-%%a"
    
    :: alpha와 beta를 소수점으로 변환
    set "alpha=0.%%a"
    set "beta=0.!beta_int!"
    
    echo Alpha: !alpha!, Beta: !beta!
    
    :: ndst 값 설정 (10에서 50까지 10씩 증가)
    for /l %%n in (10,10,50) do (
        echo Running simulation with alpha=!alpha!, beta=!beta!, ndst=%%n
        
        :: 임시 파일에 먼저 쓰기 (UTF-8 without BOM 사용)
        powershell -Command "$content = Get-Content omnetpp.ini -Encoding UTF8; $content = $content -replace '^\*\*\.drone\[\*\]\.mobility\.alpha_val = .*$', '**.drone[*].mobility.alpha_val = !alpha!'; $content = $content -replace '^\*\*\.drone\[\*\]\.mobility\.beta_val = .*$', '**.drone[*].mobility.beta_val = !beta!'; $content = $content -replace '^\*\*\.drone\[\*\]\.mobility\.ndst = .*$', '**.drone[*].mobility.ndst = %%n'; [System.IO.File]::WriteAllLines('omnetpp_temp.ini', $content)"
        
        :: 파일이 존재하는지 확인
        if exist omnetpp_temp.ini (
            :: 원본 파일 삭제
            del /f omnetpp.ini
            :: 임시 파일을 원본 이름으로 변경
            ren omnetpp_temp.ini omnetpp.ini
        )
        
        :: IDE에서 사용되는 실제 명령어로 시뮬레이션 실행
        EfficientDelivery.exe -m -u Cmdenv -c CompareAll -n .;../inet4.5/examples;../inet4.5/showcases;../inet4.5/src;../inet4.5/tests/validation;../inet4.5/tests/networks;../inet4.5/tutorials -x inet.common.selfdoc;inet.linklayer.configurator.gatescheduling.z3;inet.emulation;inet.showcases.visualizer.osg;inet.examples.emulation;inet.showcases.emulation;inet.transportlayer.tcp_lwip;inet.applications.voipstream;inet.visualizer.osg;inet.examples.voipstream --image-path=../inet4.5/images -l ../inet4.5/src/INET omnetpp.ini
        
        :: Windows의 ping 명령어로 지연
        ping -n 3 127.0.0.1 > nul
    )
)

echo All simulations completed!
pause