---
description: Code review checklist for this project
---

# Code Review Workflow

Checklist for reviewing code changes to the Smart Chess Board project.

## Before Starting

1. Read the PR/change description
2. Understand what component is being modified
3. Review relevant documentation:
   - `AGENTS.md` for project overview
   - `docs/software/architecture.md` for system design
   - Relevant feature doc in `docs/features/`

## Code Review Checklist

### General

- [ ] Code follows existing style conventions
- [ ] Comments explain "why", not "what"
- [ ] No hardcoded magic numbers (use config or constants)
- [ ] Error handling is appropriate
- [ ] Logging is sufficient for debugging

### ROS 2 Specific

- [ ] Node naming follows convention (`<package>_<function>_node`)
- [ ] Topic/service names use proper namespacing (`/<package>/...`)
- [ ] Parameters are declared with defaults
- [ ] Publishers/subscribers use appropriate QoS settings
- [ ] Node handles shutdown (cleanup callbacks)

### GPIO/Hardware

- [ ] Pin numbers match `docs/hardware/pinout.md`
- [ ] `GPIO.cleanup()` is called on shutdown
- [ ] No GPIO conflicts with existing assignments
- [ ] Appropriate delays for motor timing
- [ ] Power consumption considered

### Safety

- [ ] Limit switch checks before movement
- [ ] Emergency stop handling implemented
- [ ] No infinite loops without exit conditions
- [ ] Timeouts on blocking operations

### Configuration

- [ ] New parameters added to appropriate YAML file
- [ ] Parameter range validation present
- [ ] Documentation updated for new parameters

### Documentation

- [ ] README updated if user-facing changes
- [ ] Docstrings on public functions
- [ ] `CHANGELOG.md` updated for significant changes
- [ ] `AGENTS.md` updated if architecture changes

## Testing Requirements

### For Hardware Changes

- [ ] Tested on actual hardware (not just simulation)
- [ ] Verified with standalone script first
- [ ] Documented any calibration needed

### For Logic Changes

- [ ] Unit tests pass (`colcon test`)
- [ ] Edge cases considered (empty board, stalemate, etc.)
- [ ] State machine transitions validated

### For Vision Changes

- [ ] Tested under different lighting conditions
- [ ] Verified with various piece positions
- [ ] Performance benchmarked

## Common Issues to Watch For

| Issue | Why it Matters |
|-------|----------------|
| Missing GPIO cleanup | Leaves pins in undefined state |
| Blocking calls in ROS callbacks | Freezes the node |
| Hardcoded coordinates | Breaks with different board sizes |
| Missing error handling | Silent failures, hard to debug |
| Incorrect FEN parsing | Wrong game state, illegal moves |

## Approval Criteria

- [ ] All checklist items addressed
- [ ] Tests pass
- [ ] Documentation updated
- [ ] No regressions in existing functionality
