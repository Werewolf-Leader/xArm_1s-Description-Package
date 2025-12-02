# Alexa Interface Setup

## Prerequisites
- Amazon Developer Account
- Alexa Skill created in the Alexa Developer Console

## Configuration

1. **Copy the environment template:**
   ```bash
   cp .env.example .env
   ```

2. **Get your Alexa Skill ID:**
   - Go to https://developer.amazon.com/alexa/console/ask
   - Select your skill
   - Copy the Skill ID (format: `amzn1.ask.skill.XXXXXXXX-XXXX-XXXX-XXXX-XXXXXXXXXXXX`)

3. **Update the .env file:**
   ```bash
   nano .env
   ```
   Replace `YOUR-SKILL-ID-HERE` with your actual Skill ID

4. **Load environment variables before running:**
   ```bash
   # Option 1: Export manually
   export ALEXA_SKILL_ID=amzn1.ask.skill.YOUR-SKILL-ID-HERE
   
   # Option 2: Use python-dotenv (recommended)
   pip install python-dotenv
   ```

## Running the Alexa Interface

```bash
# Make sure environment variables are loaded
source .env  # or use python-dotenv in your code

# Run the interface
ros2 run xarm_remote alexa_interface.py
```

## Security Note
⚠️ **Never commit your `.env` file to version control!** 
The `.env` file is already in `.gitignore` to prevent accidental commits.
