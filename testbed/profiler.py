import cProfile
import framework
import vertical_stack

print('Profiler - Running main...')
print('--------------------------')
cProfile.runctx( """framework.main(vertical_stack.VerticalStack)""", globals(), locals(), filename="profile_results" )
print('---------DONE-------------')

print('------------------')
print('Profiler - Results')
print('------------------')
import pstats
p = pstats.Stats('profile_results')
p.strip_dirs().sort_stats(-1).print_stats()
print('---------DONE------')
