#!/usr/bin/env python3.9

import openai

def get_completion(prompt, model="gpt-3.5-turbo"):
    messages = [{"role": "user", "content": prompt}]
    response = openai.ChatCompletion.create(
        model=model,
        messages=messages,
        temperature=0,
    )

    return response.choices[0].message["content"] 

if __name__ == '__main__':
    prompt = "How are you doing?"
    response = get_completion(prompt)

    print(response)