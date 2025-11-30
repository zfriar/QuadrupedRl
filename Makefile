# =============================================================================
# Makefile for Quadruped RL Project
# =============================================================================
# Activate the Poetry shell before running these commands, or prefix with:
#   poetry run make <target>
# =============================================================================

# Colors
BLUE := \033[1;34m
GREEN := \033[1;32m
CYAN := \033[1;36m
YELLOW := \033[1;33m
RESET := \033[0m

.PHONY: help format lint lint-fix typecheck check precommit-install precommit-run train visualize test coverage coverage-html

# ---------------------------------------------------------------------------
# Help
# ---------------------------------------------------------------------------

help:
	@echo ""
	@echo "$(BLUE)Quadruped RL Project — Available Commands$(RESET)"
	@echo ""
	@echo "$(YELLOW)Usage:$(RESET)"
	@echo "  make <target>"
	@echo ""
	@echo "$(YELLOW)Formatting & Linting$(RESET)"
	@echo "  $(CYAN)format$(RESET)            - Format the codebase using black and isort."
	@echo "  $(CYAN)lint$(RESET)              - Run static analysis using ruff."
	@echo "  $(CYAN)lint-fix$(RESET)          - Run ruff and apply automatic fixes where possible."
	@echo "  $(CYAN)typecheck$(RESET)         - Run static type checking using mypy."
	@echo ""
	@echo "$(YELLOW)Checks & Pre-commit$(RESET)"
	@echo "  $(CYAN)code-checks$(RESET)       - Run full checks: black/isort (check), ruff, mypy, pytest."
	@echo "  $(CYAN)precommit-install$(RESET) - Install pre-commit hooks locally."
	@echo "  $(CYAN)precommit-run$(RESET)     - Run pre-commit hooks on all files."
	@echo ""
	@echo "$(YELLOW)Tests & Coverage$(RESET)"
	@echo "  $(CYAN)test$(RESET)              - Run unit tests using pytest."
	@echo "  $(CYAN)coverage$(RESET)          - Run tests and print coverage for configs, training, envs, scripts."
	@echo "  $(CYAN)coverage-html$(RESET)     - Run tests and produce an HTML coverage report (htmlcov/)."
	@echo ""
	@echo "$(YELLOW)Training & Visualization$(RESET)"
	@echo "  $(CYAN)train$(RESET)       - Train PPO on the PyBullet Ant environment."
	@echo "  $(CYAN)visualize$(RESET)   - Visualize a trained policy in PyBullet GUI."
	@echo ""

# ---------------------------------------------------------------------------
# Commands
# ---------------------------------------------------------------------------

format:  ## Format code with black and isort
	poetry run black .
	poetry run isort .

lint:  ## Run ruff static analysis
	poetry run ruff check .

lint-fix:  ## Run ruff and apply automatic fixes where possible
	poetry run ruff check --fix .

typecheck:  ## Run static type checking with mypy
	poetry run mypy .

precommit-install:  ## Install pre-commit hooks in the current repo (run once per clone)
	poetry run pre-commit install

precommit-run:  ## Run pre-commit hooks against all files (use in CI or locally)
	poetry run pre-commit run --all-files

train:  ## Train PPO agent using configs/ppo_ant.yaml
	poetry run python training/train_ppo.py --config configs/ppo_ant.yaml

visualize:  ## Visualize a trained policy
	poetry run python scripts/visualize_policy.py --model checkpoints/best_model.zip

test:  ## Run tests
	poetry run pytest -q

coverage:  ## Run tests and show coverage for configs, training, envs, scripts
	poetry run pytest --cov=configs --cov=training --cov=envs --cov=scripts \
		--cov-report=term-missing --cov-report=xml:coverage.xml -q

code-checks:  ## Run all checks via make commands (calls other make targets)
	$(MAKE) format
	$(MAKE) lint
	$(MAKE) typecheck
	$(MAKE) coverage