@echo off
rmdir /s /q .\results

for /d %%a in (.\*) do (rmdir /s /q %%a\results)
for /d %%a in (.\*) do (rmdir /s /q %%a\results_swarm)

for /d %%a in (.\parameters\*) do (rmdir /s /q %%a\results)
for /d %%a in (.\parameters\*) do (rmdir /s /q %%a\results_swarm)

for /d %%a in (.\examples\*) do (rmdir /s /q %%a\results)
for /d %%a in (.\examples\*) do (rmdir /s /q %%a\results_swarm)
echo. & pause 